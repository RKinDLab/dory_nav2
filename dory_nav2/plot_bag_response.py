import os
import pandas as pd
import numpy as np
import ast
import matplotlib.pyplot as plt

# Specify the file path
workspace_dir = os.getcwd()
# bag_name = 'dory_response_2024_06_03-19_36_38'
# bag_name = 'dory_response_2024_06_04-13_20_50'
# bag_name = 'dory_response_2024_06_04-13_52_22' # 50 N in x
# bag_name = 'dory_response_2024_06_04-18_42_53' # 50 N in x
# bag_name = 'dory_response_2024_06_04-18_58_53' # 40 N in x
# bag_name = 'dory_response_2024_06_04-19_12_38' # 0 N
# bag_name = 'dory_response_2024_06_04-19_16_15' # 5 N in x
# bag_name = 'dory_response_2024_06_04-19_21_41' # 0.1 N in x
# bag_name = 'dory_response_2024_06_04-20_07_21' # 50N in z
# bag_name = 'dory_response_2024_06_04-20_10_26' # 5N in z # thrust configuration with wilson at the beginning
# bag_name = 'dory_response_2024_06_04-20_42_59' # 5N in z # thrust configuration with wilson at the end
# bag_name = 'dory_response_2024_06_04-21_05_23' # 5N in z # thrust configuration like /dynamic_joint_states
# bag_name = 'dory_response_2024_06_05-15_35_11' # 50N in y
# bag_name = 'dory_response_2024_06_07-19_13_27' # 100N ramp in x
# bag_name = 'dory_response_2024_06_07-19_45_17' # 100N ramp in y
bag_name = 'dory_response_2024_06_07-20_01_48' # 100N ramp in z
file_path = f'{workspace_dir}/bags/{bag_name}/state_thrust_vector.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(file_path)

# Times have YYYY/MM/DD H:M:S.ns format
# ns are not zero padded and need reformating
# Separate everything before and after the period
df['string_time'] = df['time'].astype(str)
df[['DateTime', 'Nanoseconds']] = df['string_time'].str.split('.', expand=True)

# Multiply nano seconds by 10^-9 to convert to seconds
df['Nanoseconds'] = pd.to_numeric(df['Nanoseconds'])*(10**(-9))
df['Nanoseconds'] = df['Nanoseconds'].astype(str).str[2:]

# Add nano seconds back to the string
df['time'] = df['DateTime'] + '.' + df['Nanoseconds']

# Format the time %Y/%m/%d %H:%M:%S.%f
df['time'] = pd.to_datetime(df['time'], format='%Y/%m/%d %H:%M:%S.%f')

# # # Calculate the time difference between each row
# df['Time_Difference'] = df['time'].diff()
# # print(df['Time_Difference'])

# Calculate the time difference from the first row
df['TimePassed'] = (df['time'] - df['time'].iloc[0]).dt.total_seconds()
time = df['TimePassed'].to_numpy()

# convert each cell to python object (list in this case)
df['data'] = df['data'].apply(ast.literal_eval)

# convert all rows in 'data' column to nx18 array
data_array = np.array(df['data'].tolist())

# x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, force.x, force.y, force.z, torque.x, torque.y, torque.z

end_time_index = 250

# plot displacement in world coordinates
ax = plt.figure().gca(projection='3d')
ax.plot(data_array[0:end_time_index,0],
		data_array[0:end_time_index,1],
		data_array[0:end_time_index,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# plot velocity in body-fixed coordinates
plt.figure()
plt.plot(time[0:end_time_index], data_array[0:end_time_index,6], linestyle='-', label='x_dot')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,7], linestyle='--', label='y_dot')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,8], linestyle=':', label='z_dot')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

# plot angular velocity in body-fixed coordinates
plt.figure()
plt.plot(time[0:end_time_index], data_array[0:end_time_index,9], linestyle='-', label='roll_dot')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,10], linestyle='--', label='pitch_dot')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,11], linestyle=':', label='yaw_dot')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Ang. Vel. (rad/s)')

# plot thrust in body-fixed coordinates
plt.figure()
plt.plot(time[0:end_time_index], data_array[0:end_time_index,12], linestyle='-', label='F_x')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,13], linestyle='--', label='F_y')
plt.plot(time[0:end_time_index], data_array[0:end_time_index,14], linestyle=':', label='F_z')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')

plt.show()