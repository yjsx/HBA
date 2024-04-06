#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct Submap {
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    std::string data_path;
    std::string topic;

	int length;
};

std::vector<Submap> get_submap_info(std::string config_path){
	YAML::Node config = YAML::LoadFile(config_path);

	// Read the number of submaps
	int submap_num = config["submap_num"].as<int>();
	std::cout << "Number of submaps: " << submap_num << std::endl;

	// Iterate over submaps and store data into a Submap struct
	std::vector<Submap> submaps(submap_num);
	for(int i = 0; i < submap_num; ++i) {
		std::string submapKey = "submap" + std::to_string(i);
		
		// Read transformation
		const YAML::Node &transformationNode = config[submapKey]["transformation"];
		Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(); // Set to identity matrix by default
        for(int j = 0; j < transformationNode.size(); ++j) {
            for(int k = 0; k < transformationNode[j].size(); ++k) {
                transformation(j, k) = transformationNode[j][k].as<double>();
            }
        }
		Eigen::Matrix3d rotationMatrix = transformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rotation(rotationMatrix);
        Eigen::Vector3d translation(transformation.block<3, 1>(0, 3));


		// Read data path
		std::string data_path = config[submapKey]["data_path"].as<std::string>();
		std::string topic = config[submapKey]["topic"].as<std::string>();
		// Store into struct
		submaps[i].rotation = rotation;
        submaps[i].translation = translation;
		submaps[i].data_path = data_path;
		submaps[i].topic = topic;
	}

	for(const auto& submap: submaps) {
		// std::cout << submap.translation<<std::endl;
		std::cout << "Data path: " << submap.data_path  << std::endl << std::endl;
	}
	return submaps;
}