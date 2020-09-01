# Enable running this script from any directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

# Copy all models in /worlds/models to gazebo's model directory
mkdir -p ~/.gazebo/models
cp -vr models/sumo_ring ~/.gazebo/models
