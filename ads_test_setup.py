import os
os.environ['KMP_DUPLICATE_LIB_OK']="TRUE" # this is a workaround for an issue once pytorch was installed 
from time import sleep
import time
import atexit
import numpy as np
import matplotlib.pyplot as plt
import sys

sys.path.append(r'C:\\Users\\Public\\Documents\\pyripherals\\python\\src\\')

from pyripherals.utils import to_voltage, from_voltage
from pyripherals.core import FPGA, Endpoint
from pyripherals.peripherals.PIT import PIT # programmable interval timer 

# Defines data_dir_covg and adds the path to boards.py into the sys.path 
from setup_paths import *

from analysis.adc_data import read_h5
from datastream.datastream import extract_ads_data
from instruments.power_supply import open_rigol_supply, pwr_off, config_supply
from boards import Daq

INSTRUMENTS = False 

if INSTRUMENTS:
    from instrbuilder.instrument_opening import open_by_name 
    osc = open_by_name('msox_scope')

plt.ion()

def write_ddr():    
    # write channels to the DDR
    ddr.write_setup(data_driven_clock=False)
    # clear read, set write, etc. handled within write_channels
    block_pipe_return, speed_MBs = ddr.write_channels(set_ddr_read=False)
    ddr.reset_mig_interface()
    ddr.write_finish()

# sampling rates for the fast DAC, the high-speed ADC, and the ADS8686 ADC
DAC_FS = 2.5e6
FS = 5e6
SAMPLE_PERIOD = 1/FS
ADS_FS = 1e6/2

eps = Endpoint.endpoints_from_defines

if INSTRUMENTS:
    pwr_setup = "3dual"
    # -------- power supplies -----------
    dc_pwr, dc_pwr2 = open_rigol_supply(setup=pwr_setup)
    if pwr_setup == "3dual":
        atexit.register(pwr_off, [dc_pwr])
    else:
        atexit.register(pwr_off, [dc_pwr, dc_pwr2])
    # change to 16.5 if the negative regulator is still populated
    config_supply(dc_pwr, dc_pwr2, setup=pwr_setup, neg=15)

    # turn on the 7V
    dc_pwr.set("out_state", "ON", configs={"chan": 1})

    if pwr_setup != "3dual":
        # turn on the +/-16.5 V input
        for ch in [1, 2]:
            dc_pwr2.set("out_state", "ON", configs={"chan": ch})
    elif pwr_setup == "3dual":
        # turn on the +/-16.5 V input
        for ch in [2, 3]:
            dc_pwr.set("out_state", "ON", configs={"chan": ch})
    # ------------------------------------

# Initialize FPGA
f = FPGA()
f.init_device()
sleep(0.5)
f.send_trig(eps["GP"]["SYSTEM_RESET"])  # system reset

pwr = Daq.Power(f)
pwr.all_off()  # disable all power enables

daq = Daq(f)
ddr = daq.ddr
ad7961s = daq.ADC
ad7961s[0].reset_wire(1)    # Only actually one WIRE_RESET for all AD7961s

pit = PIT.create_chips(fpga=f, number_of_chips=1)[0]

#Endpoint.pit=Endpoint.get_chip_endpoints('PIT')
#pit = PIT.create_chips(fpga=f, number_of_chips=1)[0]

# pit.wb_reset()
# pit.reset()
# pit.reset_clear()
    #pit.wb_write('MOD', 100000000)
# pit.wb_write('CNT_EN', 1)
# pit.set_period(.1)
# ctrl_reg = pit.wb_read('CTRL')
#print(f'WB control reg {ctrl_reg}')

# power supply turn on via FPGA enables
for name in ["1V8", "5V", "3V3"]:
    pwr.supply_on(name)
    sleep(0.05)

# configure the SPI debug MUXs
gpio = Daq.GPIO(f)
gpio.spi_debug("ads")
gpio.ads_misc("convst")  # to check sample rate of ADS

# -------- configure the ADS8686
ads = daq.ADC_gp # note that the ADS8686 inherits from SPIcontroller so some of the methods (e.g., set_fpga_mode) are found there 

ads_voltage_range = 5  # need this for to_voltage later 
ads.hw_reset(val=True) # sets the level of the HW reset to high. would need sequence of ads.hw_reset(val=True), ads.hw_reset(val=False) to actually reset
sleep(0.001)
ads.hw_reset(val=False) # sets the level of the HW reset to high. would need sequence of ads.hw_reset(val=True), ads.hw_reset(val=False) to actually reset
ads.set_host_mode() # SPI transactions are controlled by the Python software
ads.setup()
ads.set_range(ads_voltage_range) 
ads.set_lpf(15) # 15, 39, 376

#ads.set_frequency(25)

# looks like sometimes the data is stale by one read one read 
for i in range(5):
    lpf = ads.read("lpf")
    print(f'LPF: {hex(lpf & 0xffff)}')

# this reads the register twice and returns 0x20002 should just be 0x0002 
for i in range(5):
    dev_id = ads.read("devID")
    print(f'DEV ID: {dev_id & 0xffff}')

# for i in range(21):
    # sqnrd = ads.read("seq"+str(i))
    # print('seq'+str(i)+f': {bin(sqnrd & 0xffff)}')

range_a1 = ads.read('rangeA1') # should be 5 V which is 10b for bits 7:6,5:4,3:2,1:0
range_a2 = ads.read('rangeA2') # should be 5 V which is 10b for bits 7:6,5:4,3:2,1:0

# to Test ADS8686 the sequencer channels can also be set to 
#            'AVDD', 'ALDO', 'FIXED' (0xAAAA, 0x5555)
#[('0', '1', '2', '3', '4', '5', '6', '7'), ('0', '1', '2', '3', '4', '5', '6', '7')] # 
#('0', '1'), ('0', '1')
#('0', '0'), ('1', '1'), ('2', '2')

ads_sequencer_setup = [('0', '0'), ('1', '1'), ('2', '2'), ('3', '3'), ('4', '4'), ('5', '5'), ('6', '6'), ('7', '7')] # 

# ads_sequencer_setup = [('AVDD'), ('AVDD')] 

codes = ads.setup_sequencer(chan_list=ads_sequencer_setup)
ads.write_reg_bridge() # 1 MSPS rate, sends initialization/configuration info to the FPGA SPI controller that drives the ADS8686 
# opposite is ads.set_host_mode() 
#ads.set_fpga_mode() # automatic and continuous reading to the ADS8686 that is saved into DDR 
daq.TCA[0].configure_pins([0, 0])
daq.TCA[1].configure_pins([0, 0])

# ------ Collect Data --------------
file_name = time.strftime("%Y%m%d-%H%M%S")

# --------  Enable fast ADCs  --------
for chan in [0, 1, 2, 3]:
    ad7961s[chan].power_up_adc()  # standard sampling
time.sleep(0.5)
ad7961s[0].reset_wire(0)    # Only actually one WIRE_RESET for all AD7961s
time.sleep(0.1)
ad7961s[0].reset_trig() # this IS required because it resets the timing generator of the ADS8686. Make sure to configure the ADS8686 before this reset
time.sleep(0.1)



def ads_code_to_voltage(ads_data, ads):
    data = {}
    for k in ads_data:
        data[k] = {}
        if k == 'A':
            ch_offset = 0
        elif k ==' B':
            ch_offset = 8
        for num in ads_data[k]:
            try:
                chan = ch_offset + num
            except: # for 'FIXED' and 'AVDD' channels 
                chan = ch_offset
            data[k][num] = np.array(to_voltage(ads_data[k][num], 
                                        num_bits=ads.num_bits, voltage_range=ads.ranges[chan]*2, use_twos_comp=False))
            # ads.ranges is an array for all channels.

    return data

def generate_time(data, ads, ADS_FS):
    sample_rate = ADS_FS/len(ads.sequencer_setup)
    # beware, some channels may have one fewer samples 
    return np.arange(len(data))*(1/sample_rate) #default 20/SR? why
#1/samplerate
#write_ddr()
#ddr.repeat_setup() # Get data

# dict: ads_data with arrays accessed as ads_data['A'][0] this data is in DAC codes 
#np.unique(ads_data['A'][0])
# f.read_wire_bit(ads.endpoints['OUT'].address, ads.endpoints['OUT'].bit_index_low)
# l=f.read_ep(ads.endpoints['OUT'])
# print(l)
#f.set_wire_bit(pit.endpoints['SW_POLARITY'].address, pit.endpoints['SW_POLARITY'].bit_index_low)
#clear_wire_bit sets it to zero!
