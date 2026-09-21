#!/usr/bin/env bash
# Apply DDDMR's Go2 adaptations to a freshly cloned upstream repository.
# Original README, documentation, teleop, and unused launch files are retained.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /path/to/unitree-go2-ros2" >&2
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upstream_dir="$(cd -- "$1" && pwd)"
extra_dir="$script_dir/../extra_files"
[[ -f "$upstream_dir/robots/configs/go2_config/package.xml" ]] || {
    echo "Not a unitree-go2-ros2 source tree: $upstream_dir" >&2
    exit 1
}

# Preflight extra files before changing any upstream source.
extra_files=(
    "robots/configs/go2_config/go2_config/__init__.py"
    "robots/configs/go2_config/launch/gz_lidar_odom.launch.py"
    "robots/configs/go2_config/scripts/cmd_pub_sim.py"
    "robots/configs/go2_config/scripts/gz_pose_to_ros2.py"
    "robots/configs/go2_config/scripts/note_.txt"
    "robots/configs/go2_config/worlds/slope_with_pillar_2.world"
)
for relative_path in "${extra_files[@]}"; do
    [[ -f "$extra_dir/$relative_path" ]] || {
        echo "Missing extra file: $relative_path" >&2; exit 1;
    }
    if [[ -e "$upstream_dir/$relative_path" ]] &&
       ! cmp -s "$extra_dir/$relative_path" "$upstream_dir/$relative_path"; then
        echo "Refusing to overwrite a different existing extra file: $relative_path" >&2
        exit 1
    fi
done

# 1. champ/champ_bringup/launch/bringup.launch.py:
#    Comment out the two EKF nodes in the launch list.
cd "$upstream_dir/champ/champ_bringup/launch"
sed -i \
    -e 's/^            base_to_footprint_ekf,$/            #base_to_footprint_ekf,/' \
    -e 's/^            footprint_to_odom_ekf,$/            #footprint_to_odom_ekf,/' \
    bringup.launch.py

# 2. robots/configs/go2_config/CMakeLists.txt:
#    Add Python support and install the odometry executable, once only.
cd "$upstream_dir/robots/configs/go2_config"
if ! grep -q '^find_package(ament_cmake_python REQUIRED)$' CMakeLists.txt; then
    sed -i '/^find_package(ament_cmake REQUIRED)$/a\find_package(ament_cmake_python REQUIRED)' CMakeLists.txt
fi
if ! grep -q '^ament_python_install_package' CMakeLists.txt; then
    sed -i '/^ament_package()$/i\# Install Python modules\nament_python_install_package(${PROJECT_NAME})\n\n# Install Python executables\ninstall(PROGRAMS\n  scripts/gz_pose_to_ros2.py\n  DESTINATION lib/${PROJECT_NAME}\n)\n\n' CMakeLists.txt
fi

# 3. robots/configs/go2_config/config/gait/gait.yaml:
#    Change x velocity 0.3 -> 1.0, y velocity 0.25 -> 0.5, yaw rate 0.5 -> 1.2.
cd "$upstream_dir/robots/configs/go2_config/config/gait"
sed -i \
    -e 's/^      max_linear_velocity_x : 0\.3$/      max_linear_velocity_x : 1.0/' \
    -e 's/^      max_linear_velocity_y : 0\.25$/      max_linear_velocity_y : 0.5/' \
    -e 's/^      max_angular_velocity_z : 0\.5$/      max_angular_velocity_z : 1.2/' \
    gait.yaml

# 4. robots/configs/go2_config/launch/gazebo_velodyne.launch.py:
#    Replace the default world with the slope world.
cd "$upstream_dir/robots/configs/go2_config/launch"
sed -i 's|"worlds/default.world"|"worlds/slope_with_pillar_2.world"|' gazebo_velodyne.launch.py

# 5. robots/descriptions/go2_description/xacro/gazebo.xacro:
#    Comment out only the gazebo block containing p3d_base_controller.
cd "$upstream_dir/robots/descriptions/go2_description/xacro"
sed -i '/^    <gazebo>$/{N; /name="p3d_base_controller"/s/<gazebo>/<!--gazebo>/;}' gazebo.xacro
if ! grep -q '^    </gazebo-->$' gazebo.xacro; then
    sed -i '/^    <!--gazebo>$/,/^    <\/gazebo>$/s|^    </gazebo>$|    </gazebo-->|' gazebo.xacro
fi

# Copy the additional files into their matching locations in the upstream tree.
for relative_path in "${extra_files[@]}"; do
    file_mode=644
    if [[ "$relative_path" == robots/configs/go2_config/scripts/*.py ]]; then
        file_mode=755
    fi
    install -D -m "$file_mode" "$extra_dir/$relative_path" "$upstream_dir/$relative_path"
done
echo "Go2 adaptations complete: $upstream_dir"
