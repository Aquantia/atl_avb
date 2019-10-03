# AQUANTIA AVTP Pipeline Contribution Notes

## General Status
AVTP Pipeline from https://github.com/AVnu/OpenAvnu/tree/master/lib/avtp_pipeline adapted for Aquantia’s AQC107/AQC113 Network Interface Card.

## Integrated with OpenAvnu:
- gPTP
- MAAP
- MSRP
- Build system

### Building Current OpenAvnu

Ubuntu 18.04LTS
#### Install dependencies

$ sudo apt update  
$ sudo apt install build-essential cmake  
$ sudo apt install linux-headers-generic libglib2.0-dev  
$ sudo apt install libpcap-dev libpci-dev libsndfile1-dev libjack-dev  
$ sudo apt install libasound2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev  

#### Git Submodules

After checking out the OpenAvnu git repository submodules should be configured by going:
git submodule init  
git submodule update  

Building everything
Building from the repo root  
$ make all

Binaries will be installed in lib/avtp_pipeline/build/bin.
Build files will be in the lib/avtp_pipeline/build_avdecc directory, to avoid interfering with the AVTP Pipeline build files.


The openavb_avdecc binary needs to be run in addition to the AVTP pipeline binary (openavb_harness or openavb_host) for AVDECC to be supported.


### Running OpenAvnu daemons 
Helper scripts in the repo root.  
$ sudo ./run_atl.sh eth1  
Load the atl driver  
$ sudo ./run_daemons.sh eth1  
Start the gptp, msrp, maap, and shaper daemons. Supply the interface name (ethx) as parameter.  
Daemons can also be started individually using the run_gptp.sh, run_srp.sh, run_maap.sh, and run_shaper.sh  
$ sudo ./stop_daemons.sh  
Stop the gptp, msrp, maap, and shaper daemons. Don’t use this command while AVTP Pipeline is running.  

### Running the OpenAvnu AVTP Pipeline example
$sudo ./run_avtp_pipeline.sh eth1  
Run the current OpenAvnu AVTP Pipeline example. Supply the interface name (eth1) as parameter   
$sudo ./stop_avtp_pipeline.sh eth1  
Stop the current OpenAvnu AVTP Pipeline example. The script will also attempt to cleanly recover if the AVTP pipeline binaries crashed.  

-----------------------------------