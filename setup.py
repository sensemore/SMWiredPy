##### Sensemore Wired Sensor Python Interface #######

#Required packages
# SMComPy


from setuptools import setup,find_packages


with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()



with open("./src/sensemore/SMWiredPy.py", "r", encoding="utf-8") as f:
	exec(f.readline()) # First line is version, so get it and execute it
	version = __version__  # Get the version


setup(
    name="SMWiredPy",
    version=version,
    author="sensemore",
	author_email="hello@sensemore.io",
    url="https://www.sensemore.io",
    description="Sensemore Wired Python Interface",
    long_description=long_description,
	long_description_content_type="text/markdown",
	license="MIT",
	python_requires=">=3.6",
	package_dir={"": "src"},
    packages=find_packages(where="src"),
	install_requires=[
		"pyserial==3.5",
		"pybind11==2.7.0",
		"SMComPy",
	],
	setup_requires=[
		"setuptools>=42",
		"wheel",
	]
)