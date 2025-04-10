import subprocess

# Parameters from: https://meshtastic.org/docs/overview/radio-settings/

serial_port = "/dev/ttyUSB0"
freq_start = 902000000
freq_end   = 928000000
freq_step  =    125000

spreading_factor_table = [7, 8, 9, 10, 11, 12]
bandwidth_table = [125000, 250000]
#pass a coding rate, although it doesn't really matter since LoraMon is RX only
coding_rate = 5

scan_duration = 2


#print header
print(" Frequency, Bandwidth, Spread Factor, Duration, Captured Packets")

for freq in range(freq_start, freq_end, freq_step):
    for bw in bandwidth_table:
        for sf in spreading_factor_table:
            command = ['python3', 'loramon.py']
            command.append(serial_port)
            command.append("--freq")
            command.append(str(freq))
            command.append("--bw")
            command.append(str(bw))
            command.append("--sf")
            command.append(str(sf))
            command.append("--cr")
            command.append(str(coding_rate))
            command.append("-C")
            command.append("--duration")
            command.append(str(scan_duration))
            command.append("-Q") #disable logging

            # Run the command
            try:
                #Uncomment this to see the command we run
                #print(f"{command}")
                result = subprocess.run(command)
                print("{:>{}} {:>{}} {:>{}} {:>{}} {:>{}}".format(
                                                freq, 10, 
                                                bw, 10, 
                                                sf, 14, 
                                                scan_duration, 9, 
                                                result.returncode, 14))
            except subprocess.CalledProcessError as e:
                print(f"Failed to launch {command} with error {e}")
