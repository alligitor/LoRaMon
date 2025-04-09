import subprocess

# Parameters from: https://meshtastic.org/docs/overview/radio-settings/

serial_port = "/dev/ttyUSB0"
freq_start = 902000000
freq_end   = 928000000
freq_step  =    125000

spreading_factor_table = [7, 8, 9, 10, 11, 12]
coding_rate_tabel = [5, 8]

scan_duration = 1

#print header
print(" Frequency, Bandwidth, Spread Factor, Coding Rate, Duration, Captured Packets")

for freq in range(freq_start, freq_end, freq_step):
    for sf in spreading_factor_table:
        for cr in coding_rate_tabel:
            command = ['python3', 'loramon.py']
            command.append(serial_port)
            command.append("--freq")
            command.append(str(freq))
            command.append("--bw")
            command.append(str(freq_step))
            command.append("--sf")
            command.append(str(sf))
            command.append("--cr")
            command.append(str(cr))
            command.append("-C")
            command.append("--duration")
            command.append(str(scan_duration))
            command.append("-Q") #disable logging

            # Run the command
            try:
                #Uncomment this to see the command we run
                #print(f"{command}")
                result = subprocess.run(command)
                print("{:>{}} {:>{}} {:>{}} {:>{}} {:>{}} {:>{}}".format(
                                                freq, 10, 
                                                freq_step, 10, 
                                                sf, 14, 
                                                cr, 12, 
                                                scan_duration, 9, 
                                                result.returncode, 14))
                #print(f"{freq}, {freq_step}, {sf}, {cr}, {scan_duration}, {result.returncode}")
            except subprocess.CalledProcessError as e:
                print(f"Failed to launch {command} with error {e}")
