#from ads_test_setup import *

# 500us discharge time
discharge_time = 500e-6
discharge_cnt = int(discharge_time/(1/200e6)) # = 100,000
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
check = False
integr_strt_idx = 0
integr_end_idx = 0


# CDTE_SWITCHES is stored in the DDR as dac_data[2]
#------------________--------------------------________--------------------------
#                    ^...integration period...^        
#                                              ^.......^
#                                           discharge period

# This for loop finds the first rising edge of the CDTE_SWITCHES which marks the beginning of the integration period
for i in range(len(dac_data[2])):
    if dac_data[2][i] == 0 and dac_data[2][i+1] == 65535:
        integr_strt_idx = i+1 #index of first rising edge
        print(f'start of integration found: {(integr_strt_idx/20)*(1/sample_rate)}') #1.)
        check = True 
        break
# The check signal ensures that we only index the end of a integration period after we have marked that it started
# Ensuring capture of a full integration period
if check == True:
    for i in range(len(dac_data[2][integr_strt_idx:])):
        if dac_data[2][integr_strt_idx + i] == 65535 and dac_data[2][integr_strt_idx+i+1] == 0:
            integr_end_idx = integr_strt_idx+i+1 #index of proceeding falling edge
            print(f'end of integration found: {(integr_end_idx/20)*(1/sample_rate)}') #1.)
            break
            
#1.) dac_data is sampled 20 more frequently than ads_data, therefore we must divide the integr_strt_idx by 20 to have the equivalent index of ads_data
#    Multiply by the period of ads sample rate to print the time of rising edge in seconds.



sum1=0 # Will be recording the voltage level for each channel right before the integration period ends
count1=0
for k in data:
    for i in data[k]: #2.)
        sum1+=data[k][i][int(integr_end_idx/20)-50] #3.)
        count1+=1
        #print(ads_data[k][i][int(integr_end_idx/20)-50]) #3.)
print(f'average is :{sum1/count1}')

#2.) Index i: A-B, k: 0-7, then the equivalent index of integration period end in the ads data
#    each channel 0-7 has thousands of entries. For example when numrepeats=20, len(ads_data['A'][0] = roughly 12800
#3.) I subtract by 50 to ensure valid data readings on all channels
            
# Order the dictionary to it's physical pixel location
data_new = np.empty((4,4))
data_new[0][0] = data['B'][2][int(integr_end_idx/20)-50]
data_new[0][1] = data['B'][0][int(integr_end_idx/20)-50]
data_new[0][2] = data['A'][7][int(integr_end_idx/20)-50]
data_new[0][3] = data['A'][5][int(integr_end_idx/20)-50]
 
data_new[1][0] = data['B'][3][int(integr_end_idx/20)-50]
data_new[1][1] = data['B'][1][int(integr_end_idx/20)-50]
data_new[1][2] = data['A'][6][int(integr_end_idx/20)-50]
data_new[1][3] = data['A'][4][int(integr_end_idx/20)-50]

data_new[2][0] = data['B'][4][int(integr_end_idx/20)-50]
data_new[2][1] = data['B'][6][int(integr_end_idx/20)-50]
data_new[2][2] = data['A'][1][int(integr_end_idx/20)-50]
data_new[2][3] = data['A'][3][int(integr_end_idx/20)-50]
 
data_new[3][0] = data['B'][5][int(integr_end_idx/20)-50]
data_new[3][1] = data['B'][7][int(integr_end_idx/20)-50]
data_new[3][2] = data['A'][0][int(integr_end_idx/20)-50]
data_new[3][3] = data['A'][2][int(integr_end_idx/20)-50]

plt.imshow(data_new, cmap='gray', vmin=0, vmax=3.7) 
plt.show(block=True)
  
#print(f'chan{k}{i}
t={}
ls={'A':'-','B':'--'}
#ti = generate_time(dac_data[2], ads, 1.25e6)
for k in ['A', 'B']:
    for i in range(8):
        t[i] = generate_time(data[k][i], ads, ADS_FS)
        plt.plot(t[i], data[k][i],label=f'{k}{i}',linestyle=ls[k])
    plt.axvline(x=t[0][int(integr_strt_idx/20)+50], color='b')
    plt.axvline(x=t[0][int(integr_end_idx/20)-50], color='g')
    plt.axvline(x=t[0][int(len(t[0])/2)], color='r')
plt.legend()


# plt.plot(t, data['A'][1], 'r')
plt.show(block=True)

# np.mean(data['A'][0])
# np.std(data['A'][0])

ads.set_host_mode() # stop FPGA driven SPI