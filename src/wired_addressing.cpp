#include "Senseway_WiredClient.h"


sm_err_t Senseway_WiredClient::scan(){
    using namespace MP_Senseway_to_Wired;

    /*
        1-) Integrity check if we have any device already registered
        2-) Scan for possible new devices
    */
    uint8_t max_no_devices = DEFAULT_MAX_DEVICE_NUMBER_IN_WIRED_NETWORK;

    #if(WIREDCLIENT_AP_SCREEN_OPEN==1)    
    std::vector<SM_MacAddress> custom_listed_devices;
    update_wired_device_list_from_nvs(custom_listed_devices);
    get_max_no_device_from_nvs(&max_no_devices);
    SM_LOGD("Max number of devices %u",max_no_devices);

    /*
        We could have a device from scan list, but it doesn't know who we are
        We know the mac of the device so we will assign its id first
        //get the device list
        assign id to each of them
        start integrity check and scan        
    */


    for(auto it = custom_listed_devices.begin(); it != custom_listed_devices.end(); ++it){
        
        if(device_set.find(client_device(it->as_string())) != device_set.end() ) continue; //Skip if device is already in the list!
        
        msg_auto_addressing_set_new_id sni;
        client_device cv = client_device(it->as_string(),0,DEVICE_CONNECTED);
        cv.user_defined_id = find_available_id();

        device_set.insert(cv);
        memcpy(sni.mac,it->get_native_address(),6);
        
        sni.id = cv.user_defined_id;
        
        rs485stw.write(SMCom_headers::PUBLIC_ID_4BIT, 
                        MP_Senseway_to_Wired::AUTO_ADDRESSING_SET_NEW_ID,(uint8_t*)&sni,sizeof(sni));

        vTaskDelay(5);  //Give some delay
    }
    #endif

    SM_LOGI("Integrity check for '%u' devices",device_set.size());
    for(auto it = device_set.begin(); it != device_set.end(); ){
        msg_auto_addressing_integrity_check message_integrity_check;
        memcpy(message_integrity_check.mac,it->mac.get_native_address(),6);
        uint8_t wired_id = it->user_defined_id;


        TickType_t wait_time = pdMS_TO_TICKS(1500);
        xQueueReset(rs485stw_rx_queue);
        
        for(uint8_t retry = 0; retry < 5; retry++){
            SMCom_Status_t ret = rs485stw.write(wired_id,
                                                MP_Senseway_to_Wired::AUTO_ADDRESSING_INTEGRITY_CHECK,
                                                (uint8_t*)&message_integrity_check,
                                                sizeof(MP_Senseway_to_Wired::msg_auto_addressing_integrity_check));
            if(ret == SMCOM_STATUS_SUCCESS){
                SMCOM_PUBLIC * packet = NULL;
                bool is_received = block_and_wait_wired_for_message(&packet, wired_id, MP_Senseway_to_Wired::AUTO_ADDRESSING_INTEGRITY_CHECK, wait_time);
                if(is_received == false){
                    SM_LOGI("Erasing device %s = %d",it->mac.as_string(),wired_id);
                    //If device in the network but not sent the message in the given timeout, set its id to default but don't wait
                    msg_auto_addressing_set_new_id sni;
                    memcpy(sni.mac,it->mac.get_native_address(),6);
                    sni.id = SMCom_headers::DEFAULT_ID_4BIT;
                    rs485stw.write(wired_id,MP_Senseway_to_Wired::AUTO_ADDRESSING_SET_NEW_ID,(uint8_t*)&sni,sizeof(sni));
                    vTaskDelay(10); //Wait for 10ticks
                    //Erase device
                    it = device_set.erase(it);
                }
                else{
                    SM_LOGI("Integrity checked device %s = %d\n",it->mac.as_string(),wired_id);
                    ++it;
                }
                break;
            }
            else{
                //? What if we not sent the message?, trust the retry
            }
            xQueueReset(rs485stw_rx_queue);
        }
    }

    if(device_set.size() == max_no_devices){
        return sm_err_t::SUCCESS;
    }

    //Now scanning
    xQueueReset(rs485stw_rx_queue);

    msg_auto_addressing_init message_auto_addressing;
    message_auto_addressing.delay_offset = 150;
    message_auto_addressing.channel_delay = 100; //in terms of miliseconds

    SM_LOGI("Starting WIRED scan");

    uint8_t message_start_address = SMCom_headers::DEFAULT_ID_4BIT;
    if(device_set.size() == 0){
        message_start_address = SMCom_headers::PUBLIC_ID_4BIT;
    }

    for(uint8_t retry = 0; retry<NO_MAX_RETRY_FOR_AUTO_ADDRESSING_INIT_MESSAGE; ++retry){

        message_auto_addressing.device_count = 2*(max_no_devices - device_set.size());

        TickType_t wait_time = pdMS_TO_TICKS( (message_auto_addressing.device_count * message_auto_addressing.channel_delay ) + message_auto_addressing.delay_offset );

        uint8_t wired_id = (retry != 0) ? static_cast<uint8_t>(SMCom_headers::DEFAULT_ID_4BIT) : message_start_address;

        SMCom_Status_t ret = SMCOM_STATUS_DEFAULT;

        for(uint8_t msg_sent_retry = 0; msg_sent_retry < 10; msg_sent_retry++){
            ret = rs485stw.write(  wired_id,
                                    MP_Senseway_to_Wired::AUTO_ADDRESSING_INIT,
                                    (uint8_t*)&message_auto_addressing,
                                    sizeof(MP_Senseway_to_Wired::msg_auto_addressing_init));

            if(ret == SMCOM_STATUS_PORT_BUSY || ret == SMCOM_STATUS_SUCCESS){
                break;
            }
            else{
                //Now we couldn't sent message trust retry!
            }
        }
        //Now assume that everyone is changed their id to default with the first message, when we assign new id we never sent init messages to the newly assigned devices
         

        if(ret == SMCOM_STATUS_PORT_BUSY){
            //In this case we see port busy because of collisions in the network
            //SMCom does not allow us to write anything but we will force SMCom to clear flags and we will write no matter what
            rs485stw.clear_smcom_rx_flags(); 
        }

        vTaskDelay(wait_time); //Now wait for listener task to sent messages to queue
        

        uint8_t read_amount = uxQueueMessagesWaiting(rs485stw_rx_queue);
        SM_LOGV("Incoming messages for the scan:%u", read_amount);
        while(read_amount--){
            SMCOM_PUBLIC * packet = NULL;
            BaseType_t queue_ret = xQueueReceive(rs485stw_rx_queue, &packet,0);
            if(packet->message_id != MP_Senseway_to_Wired::AUTO_ADDRESSING_INIT || queue_ret != pdPASS){
                free(packet);
                continue; //This is an error, we should not have a different message here!
            }
	        resp_auto_addressing_init * resp = (resp_auto_addressing_init*)packet->data;

            client_device wired = client_device(resp->mac, 0, DEVICE_READY);
            wired.user_defined_id = find_available_id();
            wired.version.major = resp->major;
            wired.version.minor = resp->minor;
            wired.version.patch = resp->patch;
            
	        
            msg_auto_addressing_set_new_id sni;
            memcpy(sni.mac,resp->mac,6);
            sni.id = wired.user_defined_id;
            rs485stw.write( SMCom_headers::PUBLIC_ID_4BIT,
                            MP_Senseway_to_Wired::AUTO_ADDRESSING_SET_NEW_ID,
                            (uint8_t*)&sni,
                            sizeof(msg_auto_addressing_set_new_id));
            
            SM_LOGI("Device added '%s' with ID:'%d', Version of device %u.%u.%u",wired.mac.as_string(),wired.user_defined_id,resp->major,resp->minor,resp->patch);

            device_set.insert(wired);
            free(packet);
        }
    }

    wired_led.set( device_set.size() > 0 );

    update_shared_device_set(device_set);

    SM_LOGI("Finishing WIRED scan");

    return sm_err_t::SUCCESS;
}

uint8_t Senseway_WiredClient::find_available_id(){
    //Find the available slot between 0-MAX_DEVICE_NUMBER_IN_WIRED_NETWORK

    uint16_t flag = 0x00; //16bit data, since max device is smaller than 16 we can use it safely

    //Now set the bits if we have a device at that bit
    for(auto it = device_set.begin(); it != device_set.end(); ++it){
        flag |= (0x01 << it->user_defined_id);
    }

    //Find the first zero bit
    for(uint8_t i = 0; i < device_set.size(); ++i){
        if( ( (flag>>i) & 0x01) == 0){
            return i;
        }
    }

    //If we couldn't find it probably list size is zero, or just return the list size
    return device_set.size();
}