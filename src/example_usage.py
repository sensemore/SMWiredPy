from wired import *
import time

dev = Wired()
time.sleep(.2)
print("-"*60)

dev.device_check()

print(len(dev.device_set),"devices found in network")
print("#"*60)
print()
time.sleep(1)

def x():
    for i in range(1000):
        print(i, dev.measure(0, '16G', 12800, 20000))

# dev.continue_thread = False
# dc = {'CA:B8:31:00:00:71':[], 'CA:B8:31:00:00:55':[], 'CA:B8:31:00:00:33':[], 'CA:B8:31:00:00:3C':[]}
# it = 0

# def x():
#     global it
#     it += 1
#     for k in dev.id_list:
#         dev.write(k, SMCOM_WIRED_MESSAGES.GET_VERSION.value, [], 0)
#         start_time = time.perf_counter_ns()
#         while True:
#             if dev.ser.in_waiting > 9:
#                 end_time = time.perf_counter_ns()
#                 dev.ser.read()
#                 break
#         if it > 5:
#             for i in dev.device_set:
#                 if dev.device_set[i].user_defined_id == k:
#                     if it == 1:
#                         print(i[-2:])
#                     dc[i].append(end_time - start_time)

# for i in range(100):
#     x()
# ls = [[], [], [], []]
# for i in dc:
#     ls

# dev.continue_thread = True
