#from ads_test_setup import *
from mpl_toolkits.mplot3d import Axes3D

num_repeats = 200

def capture_data(idx=0, filename=None):
    ddr.repeat_setup() # Get data
    time.sleep(2)
    if filename is None:
        filename = file_name.format(idx) + '.h5'

    # saves data to a file; returns to the workspace the deswizzled DDR data of the last repeat
    chan_data_one_repeat = ddr.save_data(data_dir, filename, num_repeats,
                                        blk_multiples=200)  # blk multiples must be multiple of 10 
    # each block multiple is 256 bytes 

    # to get the deswizzled data of all repeats need to read the file
    _, chan_data = read_h5(data_dir, file_name=filename, chan_list=np.arange(8))
    
    # Long data sequence -- entire file
    adc_data, timestamp, dac_data, ads_data_tmp, ads_seq_cnt, read_errors = ddr.data_to_names(chan_data)
    ads_data = extract_ads_data(ads_data_tmp, ads_seq_cnt, ads.sequencer_setup)
    log_info = {'timestamp_step': timestamp[1]-timestamp[0],
                'timestamp_span': 5e-9*(timestamp[-1] - timestamp[0]),
                'read_errors': read_errors}
    
    data = ads_code_to_voltage(ads_data, ads)

    return data, ads_data, log_info, ads_seq_cnt, dac_data, chan_data

pit_charge_duration = 0.1
pit_discharge_duration = 500e-6

pit.wb_reset()
pit.reset()
pit.reset_clear()
#pit.wb_write('MOD', 100000000)
pit.wb_write('CNT_EN', 1)
pit.set_period(pit_charge_duration)
ctrl_reg = pit.wb_read('CTRL')

# 500us discharge time
discharge_cnt = int(pit_discharge_duration/(1/200e6)) # = 100,000
# discharge_cnt_ep_scale = discharge_cnt/10000

f.clear_wire_bit(pit.endpoints['SW_POLARITY'].address, pit.endpoints['SW_POLARITY'].bit_index_low)
f.set_wire(eps['PITDISCHARGE']['DATA'].address, discharge_cnt, 0xffffffff)
# f.read_wire(pit.endpoints['SW_POLARITY'].address)
# f.read_wire(pit.endpoints['DISCHARGE_CNT'].address)
# f.read_wire(pit.endpoints['WIRE_IN'].address)

#%run -i
 # 1 MSPS rate, sends initialization/configuration info to the FPGA SPI controller that drives the ADS8686 


write_ddr()


ddr.repeat_setup() # Get data


ads.write_reg_bridge()

data, ads_data, log_info, sqn, dac_data, chan_data  = capture_data(idx=0) #dac_data
data, ads_data, log_info, sqn, dac_data, chan_data  = capture_data(idx=0) #dac_data


sample_rate = ADS_FS/len(ads.sequencer_setup)



# CDTE_SWITCHES is stored in the DDR as dac_data[2]
#------------________---------------------------------________--------------------------
#                    ^...integration/charge period...^        
#                                                     ^.......^
#                                                   discharge period

#arrays of vals to index ads_data
interval_start_indexes = []
interval_end_indexes = []
interval_start_times = []
interval_end_times = []
interval_lengths = []
interval_durations = []
index_offset = 50       #Acts like a setup time to allow signals to stablize after integration start and before integration end

for i in range(len(dac_data[2])):
    #records indexes of start integration
    if dac_data[2][i-1] == 0 and dac_data[2][i] == 65535:
        interval_start_indexes.append(int((i+index_offset)/20)+25) #We must /20 because there are 20x more data points in dac_data[2] than in ads_data[A-B][0-7]
    #records indexes of end integration
    if dac_data[2][i-1] == 65535 and dac_data[2][i] == 0:
        interval_end_indexes.append(int((i-index_offset)/20)-25)
    #Length of each integration interval. Records an array of interval length in seconds for each interval 
    # Len of array = len(interval_start) = len(interval_end)

#Makes sure strt_idx[0] and end_idx[0] are from the same sample
if interval_end_indexes[0] < interval_start_indexes[0]:
    interval_end_indexes.pop(0)
num_intervals = int(len(interval_end_indexes))
if len(interval_start_indexes) > num_intervals:
    interval_start_indexes.pop(len(interval_start_indexes)-1)
    
interval_start_times = [index * (1 / sample_rate) for index in interval_start_indexes]
interval_end_times = [index * (1 / sample_rate) for index in interval_end_indexes]


for i in range(num_intervals):
    interval_lengths.append(interval_end_indexes[i]-interval_start_indexes[i]) #interval length in index width
    interval_durations.append(interval_lengths[i]*(1/sample_rate))   #interval length in seconds
    
    
t = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
start_vals = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
end_vals = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
interval_differences = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
slopes = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
currents = {k: {i: [] for i in range(8)} for k in ['A', 'B']}
ave_currents = {k: {i:None for i in range(8)} for k in ['A', 'B']}

#t = dict of dicts of arrays [A-B]:[0-7]:[chan_time_array[0:]]
#start_values = dict of dicts of arrays [A-B]:[0-7]:[strt_idxs[0:]] <- array of start integration indexes
#end_values   = dict of dicts of arrays [A-B]:[0-7]:[end_idxs[0:]]  <- array of end integration indexes

#-----combine?--------
for k in data:
    for i in data[k]:
        #dict of time arrays for each chan. Len of time array = len data
        t[k][i] = generate_time(data[k][i], ads, ADS_FS)
        #dict of chan values at start integration
        start_vals[k][i] = data[k][i][interval_start_indexes]
        #dict of chan values at end integration
        end_vals[k][i] = data[k][i][interval_end_indexes]
for k in data:
    for i in data[k]:
        current_sum=0
        for g in range(num_intervals):
            interval_differences[k][i].append(end_vals[k][i][g]-start_vals[k][i][g])
            slopes[k][i].append(interval_differences[k][i][g]/interval_durations[g])
            currents[k][i].append(slopes[k][i][g]*(33e-12)*1e9) #nA
            current_sum+=currents[k][i][g]
        ave_currents[k][i]=current_sum/num_intervals
#-----combine?--------


    
 
#Calculate current for each channel for each integration interval.


#1.) dac_data is sampled 20 more frequently than ads_data, therefore we must divide the interval_start by 20 to have the equivalent index of ads_data
#    Multiply by the period of ads sample rate to print the time of rising edge in seconds.

# sum1=0 # Will be recording the voltage level for each channel right before the integration period ends
# count1=0
# for k in data:
    # for i in data[k]: #2.)
        # sum1+=data[k][i] #3.)
        # count1+=1
        #print(ads_data[k][i]) #3.)
# print(f'average is :{sum1/count1}')

#2.) Index i: A-B, k: 0-7, then the equivalent index of integration period end in the ads data
#    each channel 0-7 has thousands of entries. For example when numrepeats=20, len(ads_data['A'][0] = roughly 12800
#3.) I subtract by 50 to ensure valid data readings on all channels
            
# Order the dictionary to it's physical pixel location
data_new = np.empty((4,4))
# best to do ave_end_voltages across all intervals so we know max brightness will be 3.7
data_new[0][0] = ave_currents['B'][2]
data_new[0][1] = ave_currents['B'][0]
data_new[0][2] = ave_currents['A'][7]
data_new[0][3] = ave_currents['A'][5]
 
data_new[1][0] = ave_currents['B'][3]
data_new[1][1] = ave_currents['B'][1]
data_new[1][2] = ave_currents['A'][6]
data_new[1][3] = ave_currents['A'][4]

data_new[2][0] = ave_currents['B'][4]
data_new[2][1] = ave_currents['B'][6]
data_new[2][2] = ave_currents['A'][1]
data_new[2][3] = ave_currents['A'][3]
 
data_new[3][0] = ave_currents['B'][5]
data_new[3][1] = ave_currents['B'][7]
data_new[3][2] = ave_currents['A'][0]
data_new[3][3] = ave_currents['A'][2]

plt.figure(1)
plt.imshow(data_new, cmap='gray', vmin=0, vmax=1.2) 
plt.show(block=False)
  
#print(f'chan{k}{i}
#t={}
ls={'A':'-','B':'--'}
# plt.figure(2)
#ti = generate_time(dac_data[2], ads, 1.25e6)
# for k in ['A', 'B']:
    # for i in range(8):
        #t[k][i] = generate_time(data[k][i], ads, ADS_FS)
        # plt.plot(t[k][i], data[k][i], label=f'{k}{i}', linestyle=ls[k])
# for i in range(len(interval_start_indexes)):
    # plt.axvline(x=t['A'][0][interval_start_indexes[i]], color='b')
    # plt.axvline(x=t['A'][0][interval_end_indexes[i]], color='g')
#plt.legend()

colors = [
    "#FF0000",  # Red
    "#FF7F00",  # Orange
    "#FFFF00",  # Yellow
    "#00FF00",  # Green
    "#0000FF",  # Blue
    "#4B0082",  # Indigo
    "#9400D3",  # Violet
    "#FF00FF"   # Magenta (extra color for balance)
]
markers={'A':'o','B':'s'}
#plot channel current across each interval
plt.figure(3)
for k in ['A', 'B']:
    for i in range(8):
        for g in range(num_intervals):
            plt.plot(g, currents[k][i][g], marker=markers[k], color=colors[i])#,linestyle=ls[k])
plt.show(block=False)


#plt.plot(t, data['A'][1], 'r')
plt.show(block=False)

np.mean(data['A'][0])
np.std(data['A'][0])

ads.set_host_mode() # stop FPGA driven SPIl