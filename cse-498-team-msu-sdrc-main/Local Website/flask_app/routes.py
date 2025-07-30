from flask import current_app as app, jsonify, render_template, redirect, request
import os
import json
import subprocess  

#for command line interactions(record/replay function)
from flask import Flask
import os
import signal
import subprocess
import pty
import time
import datetime


#CHANGE THIS TO YOUR OWN PATH IN ORDER FOR RECORD AND REPLAY TO WORK
home_path = "/home/vu"

UPLOAD_FOLDER = 'replay_sessions'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

#this is global variable that will be used to to find the folder to store the recordings
session_folder_name = None

@app.route('/')
def root():
    return redirect('/home')

#This is what makes the heatmap have a dynamic input (live changes)
#once it retreives the html form via post request it runs a command via the command line to set a new parameter
#onto my ros2 heat map publisher which changes the distance
@app.route('/home', methods=['GET', 'POST'])
def home():
    status_msg = ""
    
    if request.method == 'POST':
        try:
            new_span = float(request.form['span'])

            result = subprocess.run(
                ["ros2", "param", "set", "/depth_heatmap_only_node", "span", str(new_span)],
                capture_output=True,
                text=True
            )

            if result.returncode == 0:
                status_msg = f"Span updated to {new_span}"
            else:
                status_msg = f"Failed: {result.stderr}"

        except Exception as e:
            status_msg = f"Exception: {e}"

    return render_template('home.html', status_msg=status_msg)


#this handles the file creations and paths for the file selection button
@app.route('/upload', methods=['POST'])
def upload():
    global session_folder_name

    uploaded_file = request.files['file']
    rel_path = uploaded_file.filename 

    parts = rel_path.split('/')
    session_folder_name = parts[0]
    print("Session folder name:", session_folder_name)

    save_path = os.path.join(UPLOAD_FOLDER, rel_path)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    uploaded_file.save(save_path)

    return jsonify({
        "message": f"Saved to {rel_path}",
        "session": session_folder_name
    })



bagProcess = {}


@app.route('/playROS2Bag', methods = ['POST'])
def playROS2Bag():
	global bagProcess
	data = request.get_json()
	isReplayMode = data.get("isReplayMode", False)
	bagToplay = session_folder_name #"2025-03-31_00-14-26"

	if isReplayMode:
		cmd = (
			"ps aux | grep '[r]os2 bag record' | awk '{print $2}' | xargs sudo kill -SIGINT"
		)
		subprocess.run(cmd, shell = True, check = False)
		print("recording stop cmd sent")

		if bagProcess:
			for process in bagProcess.values():
				process.send_signal(signal.SIGINT)
				#bagProcess.terminate()

			bagProcess.clear()

		bags =[
			"gnss_bag",
			"lidar_bag",
			"imu_bag",
			"camera_bag",
			"controller_bag",
			]
		
		topics =[
			"/camera_feed",
			"/controller_data"
			"/scan"
			"/gnss"
			"/imu/data_raw"
		]

		fixed_dir = f"{home_path}/Desktop/ros2_bags"

		# for bag in bags:
		# 	bagPath = os.path.join(fixed_dir,bagToplay,bag)

		# 	cmd = ('sudo bash -c "'
		# 		'source {home_path}/ros2_humble/install/setup.bash; '
		# 		'source {home_path}/ros2_ws/install/setup.bash; '
		# 		f'exec ros2 bag play {bagPath}"')
		# 	process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
		# 	bagProcess[bag] = process

		bagPath = os.path.join(fixed_dir,bagToplay,"ros2_bag")

		cmd = ('sudo bash -c "'
			f'source {home_path}/ros2_humble/install/setup.bash; '
			f'source {home_path}/ros2_ws/install/setup.bash; '
			f'exec ros2 bag play {bagPath}"')
		process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
		bagProcess["ros2_bag"] = process

		response = {"status": "bag started"}

	else:
		if bagProcess:
			#for process in bagProcess.values():
				#process.send_signal(signal.SIGINT)
				#process.wait()
				#bagProcess.terminate()

				#pgid = os.getpgid(process.pid)
				#cmd = f"sudo kill -9 -{pgid}"
				#subprocess.Popen(cmd, shell=True)
			try:
				stop_cmd = (
					"ps aux | grep '[r]os2 bag play' | awk '{print $2}' | xargs sudo kill -SIGINT"
				)

				subprocess.run(stop_cmd, shell=True, check=False)
				response = {"status": "all ros2 bag playback stopped"}
			except Exception as e:
				print(f"[ERROR] Failed to stop rosbag: {e}")
				response = {"status": "error stopping bags", "error": str(e)}	

			bagProcess.clear()
			#response = {"status": "bag stopped"}
		else:
			response = {"status": "no bag is running"}
    
	return jsonify(response), 200



	# ros bag location for testing:
	# {home_path}/Desktop/rosbag2_2025_03_20-01_59_09



recordProcess = {}
relayProcess = {}
recordFolder = None

@app.route('/recordROS2Bag', methods = ['POST'])
def recordRos2Bag():
	global recordProcess, recordFolder,relayProcess
	data = request.get_json()
	isRecording = data.get("isRecordMode", False)
	topicsToRecord =[
			("/gnss", "gnss_bag"),
			("/scan", "lidar_bag"),
			("/imu/data_raw", "imu_bag"),
			("/camera_feed","camera_bag"),
			("/controller_data", "controller_bag"),
			]
	if isRecording:
		#print('here we go')
		if recordProcess:
			return jsonify({"status": "recording"}), 400
		
		fixed_dir = f"{home_path}/Desktop/ros2_bags"
		timeAtRecord = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
		recordFolder = os.path.join(fixed_dir, timeAtRecord)
		os.makedirs(recordFolder,exist_ok=True)

		relayProcess = {}
		for(topic, folder) in topicsToRecord:
			pathToFolder = os.path.join(recordFolder,folder)

			rlayCmd = ('sudo bash -c "'
				   f'source {home_path}/ros2_humble/install/setup.bash; '
				   f'source {home_path}/ros2_ws/install/setup.bash; '
				   f'ros2 run topic_tools relay {topic} {topic}_bag"')
			process = subprocess.Popen(
									   rlayCmd,
									   shell=True,
									   preexec_fn=os.setsid,
									  )
			relayProcess[folder] = process


		# recordProcess ={}
		
		# for(topic, folder) in topicsToRecord:
		# 	pathToFolder = os.path.join(recordFolder,folder)

		# 	cmd = ('sudo bash -c "'
		# 		   'source {home_path}/ros2_humble/install/setup.bash; '
		# 		   'source {home_path}/ros2_ws/install/setup.bash; '
		# 		   f'cd {recordFolder} && ros2 bag record {topic}_bag -o {folder}"')
		# 	process = subprocess.Popen(
		# 							   cmd,
		# 							   shell=True,
		# 							   preexec_fn=os.setsid,
		# 							  )
		# 	recordProcess[folder] = process

		# return jsonify({"status": "recording started", "folder": recordFolder}), 200

		topics = "/gnss_bag /scan_bag /imu/data_raw_bag /camera_feed_bag /controller_data_bag"

		cmd = ('sudo bash -c "'
				f'source {home_path}/ros2_humble/install/setup.bash; '
				f'source {home_path}/ros2_ws/install/setup.bash; '
				f'cd {recordFolder} && ros2 bag record {topics} -o ros2_bag"')
		
		recordProcess = subprocess.Popen(
								   cmd,
								   shell=True,
								   preexec_fn=os.setsid,
								  )
		return jsonify({"status": "recording started", "folder": recordFolder}), 200
	
	else:
		if not recordProcess:
			return("no bag is running")

		"""
		for folder,process in recordProcess.items():
				os.killpg(os.getpgid(process.pid), signal.SIGINT)
		
		for folder,process in recordProcess.items(): 
			try:
				print('2')
				process.wait(timeout=0.5)
			except subprocess.TimeoutExpired:
				print('3')
				os.killpg(os.getpgid(process.pid), signal.SIGKILL)
				process.wait(timeout=3)
		"""
		try:
			cmd = (
				"ps aux | grep '[r]os2 bag record' | awk '{print $2}' | xargs sudo kill -SIGINT"
			)
			subprocess.run(cmd, shell = True, check = False)
			print("recording stop cmd sent")
			
			time.sleep(5)

			stopCmd = ("sudo pkill -9 -f 'ros2 bag record'")
			subprocess.run(stopCmd, shell = True, check = False)
			

			relayCmd = ("sudo pkill -f topic_tools")
			subprocess.run(relayCmd, shell = True, check = False)
			print("recording stop cmd sent")
			stopSuccessful = True

			subprocess.run("stty sane", shell=True, check=False)

		except Exception as e:
			print("failed to stop")
			stopSuccessful = False
			# for folder,process in recordProcess.items():
			# 	os.killpg(os.getpgid(process.pid), signal.SIGINT)
		
			# for folder,process in recordProcess.items(): 
			# 	try:
			# 		print('2')
			# 		process.wait(timeout=0.5)
			# 	except subprocess.TimeoutExpired:
			# 		print('3')
			# 		os.killpg(os.getpgid(process.pid), signal.SIGKILL)
			# 		process.wait(timeout=3)

			# for topic, process in relayProcess.items():
			# 	try:
			# 		os.killpg(os.getpgid(process.pid), signal.SIGINT)
			# 		process.wait(timeout=0.5)
			# 	except subprocess.TimeoutExpired:
			# 		os.killpg(os.getpgid(process.pid), signal.SIGKILL)
			# 		process.wait(timeout=3)
        
		relayProcess.clear()
		
		recordProcess = None
		
		allRecordingFinished = False

		if not stopSuccessful:
			for (topic,folder) in topicsToRecord:
				pathToFolder = os.path.join(recordFolder,folder)
				reindexCmd = (
						'sudo bash -c "'
						f'source {home_path}/ros2_humble/install/setup.bash; '
						f'source {home_path}/ros2_ws/install/setup.bash; '
						f'ros2 bag reindex {pathToFolder}"'
						)
				process = subprocess.Popen(reindexCmd, 
										shell=True, 
										stdout=subprocess.PIPE, 
										stderr=subprocess.PIPE
										)
				out, err = process.communicate()

				if process.returncode != 0:
					allRecordingFinished = True

			if allRecordingFinished:
				return jsonify({
						"status": "recording stopped, but reindex failed",
						"error": err.decode("utf-8")
				}), 500
			else:
				return jsonify({
						"status": "recording stopped, reindex complete",
						"folder": recordFolder
				}), 200

		return jsonify({"status": "no recording running"}), 200


@app.route('/pauseResume', methods=['POST'])
def pauseResume():
    cmd = (f'sudo bash -c "source {home_path}/ros2_humble/install/setup.bash && '
         'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused \'{}\'"'
		 )
		#'sudo bash -c "source {home_path}/ros2_humble/install/setup.bash && '
        #  'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused \'{}\'"'

		#"ps aux | grep '[p]ython3 /opt/ros/humble/bin/ros2 bag play' | awk '{print $2}' | xargs sudo kill -SIGINT"
		

    try:
        subprocess.run(cmd, shell=True, check=True)
        return jsonify({"status": "toggled"}), 200
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500


@app.route('/start_ros', methods=['POST'])
def start_ros():
    try:
        # Define the terminal command to start ROS nodes (first command)
        command1 = f"source {home_path}/ros2_humble/install/setup.bash; source {home_path}/ros2_ws/install/setup.bash; ros2 launch start_laptop start_laptop_launch.py"
        
        # Use gnome-terminal to execute the first command
        terminal_command1 = f"gnome-terminal -- bash -c 'echo Random123 | sudo -S bash -c \"{command1}; exec bash\"'"

        # Define the second command (ROS bridge command)
        command2 = f"cd {home_path}/local_web/cse-498-team-msu-sdrc && source {home_path}/ros2_humble/install/setup.bash && source {home_path}/ros2_ws/install/setup.bash && ros2 run rosbridge_server rosbridge_websocket"
        
        # Use gnome-terminal to execute the second command
        terminal_command2 = f"gnome-terminal -- bash -c 'echo Random123 | sudo -S bash -c \"{command2}; exec bash\"'"

        # command3 = f"sudo bash -c 'source {home_path}/ros2_humble/install/setup.bash; python3 temp2/cse-498-team-msu-sdrc/ROS2\ Code/src/loopback_test/loopback_test_laptop.py'"

        # terminal_command3 = f"gnome-terminal -- bash -c 'echo Random123 | sudo -S bash -c \"{command3}; exec bash\"'"

        # Run both commands concurrently
        result1 = subprocess.run(terminal_command1, shell=True, text=True, capture_output=True)
        result2 = subprocess.run(terminal_command2, shell=True, text=True, capture_output=True)
        # result3 = subprocess.run(terminal_command3, shell=True, text=True, capture_output=True)

        # Check if there was an error in the execution of the first command
        if result1.returncode != 0:
            return jsonify({"status": "error", "message": result1.stderr}), 500
        
        # Check if there was an error in the execution of the second command
        if result2.returncode != 0:
            return jsonify({"status": "error", "message": result2.stderr}), 500
		
        # if result3.returncode != 0:
        #     return jsonify({"status": "error", "message": result2.stderr}), 500

        # If both commands are successful, return a success response
        return jsonify({"status": "success", "message": "ROS nodes and ROS bridge started in two separate terminals!"}), 200


    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)
