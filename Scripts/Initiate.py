import subprocess
import signal
import sys
import os
import time

def main():
    # 1) Figure out where everything lives
    script_dir = os.path.dirname(os.path.realpath(__file__))
    parent_dir = os.path.abspath(os.path.join(script_dir, ".."))

    px4_dir = os.path.join(parent_dir, "Applications", "PX4-Autopilot")
    rviz_config_path = os.path.join(parent_dir, "Scripts", "config.rviz")
    qgc_path = os.path.join(parent_dir, "Applications", "QGroundControl.AppImage")

    # 2) Build the launcher commands
    # PX4 SITL (runs in THIS terminal—no gnome-terminal)
    px4_cmd = ["make", "px4_sitl", "gz_x500_mono_cam_baylands"]

    # ROS–Gazebo Bridge (spawn in its own GNOME Terminal)
    bridge_params = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        # camera topics...
        "/world/baylands/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/world/baylands/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/world/baylands/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/world/baylands/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/world/baylands/model/x500_mono_cam_0/model/down_cam_left/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/world/baylands/model/x500_mono_cam_0/model/down_cam_left/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/world/baylands/model/x500_mono_cam_0/model/down_cam_right/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/world/baylands/model/x500_mono_cam_0/model/down_cam_right/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "--ros-args",
        # remappings
        "-r", "/world/baylands/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/image:=/camera/front_left/image",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/front_left_cam/link/camera_link/sensor/imager/camera_info:=/camera/front_left/camera_info",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/image:=/camera/front_right/image",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/front_right_cam/link/camera_link/sensor/imager/camera_info:=/camera/front_right/camera_info",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/down_cam_left/link/camera_link/sensor/imager/image:=/camera/down_left/image",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/down_cam_left/link/camera_link/sensor/imager/camera_info:=/camera/down_left/camera_info",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/down_cam_right/link/camera_link/sensor/imager/image:=/camera/down_right/image",
        "-r", "/world/baylands/model/x500_mono_cam_0/model/down_cam_right/link/camera_link/sensor/imager/camera_info:=/camera/down_right/camera_info"
    ]
    bridge_cmd_str = " ".join(bridge_params) + "; exec bash"

    # MAVROS (own terminal)
    mavros_params = [
        "ros2", "run", "mavros", "mavros_node",
        "--ros-args", "-p", "fcu_url:=udp://:14540@127.0.0.1:14557"
    ]
    mavros_cmd_str = " ".join(mavros_params) + "; exec bash"

    # RViz2 (own terminal)
    rviz_cmd_str = f"rviz2 -d {rviz_config_path}; exec bash"

    # QGroundControl (own terminal)
    qgc_cmd_str = f"{qgc_path}; exec bash"

    # 3) Launch PX4 SITL directly (no extra terminal). It inherits the same process group
    print("🛫 Launching PX4 SITL in THIS terminal…")
    px4_proc = subprocess.Popen(
        px4_cmd,
        cwd=px4_dir
        # no preexec_fn, so it inherits this terminal’s PGID → Ctrl+C hits both
    )
    print(f"  • PX4 SITL PID: {px4_proc.pid}")

    # Give PX4 SITL about 20 s to boot
    time.sleep(20)

    # 4) Now launch the other nodes in their own GNOME Terminal windows (with separate PGIDs)
    print("🔥 Spawning Bridge, MAVROS, RViz2 & QGC in separate terminals…")
    terminals = []  # will hold the Popen objects for the gnome-terminal calls

    def spawn_terminal(cmd_str):
        proc = subprocess.Popen(
            ["gnome-terminal", "--disable-factory", "--", "bash", "-c", cmd_str],
            preexec_fn=os.setpgrp  # ensures this terminal (and its children) have a distinct PGID
        )
        terminals.append(proc)
        return proc

    spawn_terminal(bridge_cmd_str)
    spawn_terminal(mavros_cmd_str)
    spawn_terminal(rviz_cmd_str)
    spawn_terminal(qgc_cmd_str)

    # 5) Define a shutdown handler that:
    #    a) sends SIGINT to px4_proc (since it’s in the same PGID as us, we can just sys.exit() too)
    #    b) sends SIGINT to each gnome-terminal PGID → wait 2 s → SIGKILL if needed
    def shutdown(sig, frame):
        print("\n🛑 Ctrl+C detected—shutting down everything…")

        # 5a) PX4 SITL is a direct child in our PGID, so it already gets SIGINT when we exit.
        # But let’s explicitly send SIGINT to it, just in case:
        try:
            px4_proc.send_signal(signal.SIGINT)
        except Exception as e:
            print(f"  • couldn’t SIGINT PX4 (PID {px4_proc.pid}): {e}")

        # 5b) Send SIGINT to each gnome-terminal’s PGID
        for term in terminals:
            try:
                os.killpg(term.pid, signal.SIGINT)
            except Exception as e:
                print(f"  • couldn’t SIGINT PGID {term.pid}: {e}")

        # Wait 2 s for everything to die gracefully
        time.sleep(2)

        # 5c) Force-kill any leftovers in those PGIDs
        for term in terminals:
            try:
                os.killpg(term.pid, signal.SIGKILL)
            except Exception as e:
                print(f"  • couldn’t SIGKILL PGID {term.pid}: {e}")

        # Finally, exit this launcher too
        sys.exit(0)

    # Catch Ctrl+C on the launcher
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # 6) Block here, so the script doesn’t quit immediately.
    #    If PX4 SITL gets SIGINT (because we exit), the launcher will exit too.
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()

