#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <pcl/common/io.h>


void CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr,
                  float *out_points_array, const float normalizing_factor) {
    for (size_t i = 0; i < cloud_ptr->size(); i++) {
        pcl::PointXYZI point = cloud_ptr->at(i);
        out_points_array[i * 4 + 0] = point.x;
        out_points_array[i * 4 + 1] = point.y;
        out_points_array[i * 4 + 2] = point.z;
        out_points_array[i * 4 + 3] = point.intensity / normalizing_factor;
    }
}


int loadData(const char *file, void **data, unsigned int *length) {
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open()) {
        std::cout << "Can't open files: " << file << std::endl;
        return -1;
    }

    // get length of file:
    unsigned int len = 0;
    dataFile.seekg(0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg(0, dataFile.beg);

    // allocate memory:
    char *buffer = new char[len];
    if (buffer == NULL) {
        std::cout << "Can't malloc buffer." << std::endl;
        dataFile.close();
        exit(-1);
    }

    // read data as a block:
    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void *)buffer;
    *length = len;
    return 0;
}

void readBinFile(std::string &filename, void *&bufPtr, int &pointNum,
                 int pointDim) {
    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    if (!file) {
        std::cerr << "[Error] Open file " << filename << " failed" << std::endl;
        return;
    }

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    bufPtr = malloc(fileSize);
    if (bufPtr == nullptr) {
        std::cerr << "[Error] Malloc Memory Failed! Size: " << fileSize
                  << std::endl;
        return;
    }
    // read the data:
    file.read((char *)bufPtr, fileSize);
    file.close();

    pointNum = fileSize / sizeof(float) / pointDim;
    if (fileSize / sizeof(float) % pointDim != 0) {
        std::cerr << "[Error] File Size Error! " << fileSize << std::endl;
    }
    std::cout << "[INFO] pointNum : " << pointNum << std::endl;
}


int main(int argc, char **argv) {
	std::ifstream in_file("../config/test.txt");
	std::string data_package = "/media/xiaohu/xiaohu/4d毫米波/view_of_delft_PUBLIC/radar_3frames/training/velodyne/";
    if (!in_file) {
        std::cout << "data_package is not exist  " << std::endl;
        return 0;
    }

	std::string name;
	// while (in_file >> name) {
    //     // std::string index_str = name.substr(0, 6);
    //     // std::cout << "index_str " << index_str << std::endl;

    //     std::string data_file = data_package + name;
	// 	// 每一帧包括 x,y,z,RCS，相对径向速度，补偿后的相对径向速度（绝对径向速度），以及相对时间ID
    //     data_file += ".bin";

    //     // load rader points cloud
    //     unsigned int length = 0;
    //     void *data = NULL;
    //     std::shared_ptr<char> buffer((char *)data,std::default_delete<char[]>());
    //     loadData(data_file.data(), &data, &length);
    //     buffer.reset((char *)data);
    //     float *points = (float *)buffer.get();
    //     size_t points_size = length / sizeof(float) / 7;

	// 	std::cout << "rader point size : " << points_size << std::endl;
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	// 	float * data_ptr = static_cast<float *>(data);
	// 	for(int i=0; i< points_size ;i++){
    //         pcl::PointXYZ p;
    //         p.x = data_ptr[7*i+0];
    //         p.y = data_ptr[7*i+1];
    //         p.z = data_ptr[7*i+2];	
	// 		in_cloud_ptr->push_back(p);

    //         printf("%f %f %f %f %f %f %f \n",data_ptr[7*i+0],data_ptr[7*i+1],data_ptr[7*i+2],data_ptr[7*i+3],data_ptr[7*i+4],data_ptr[7*i+5],data_ptr[7*i+6]);
	// 	}
    // }

        std::string data_file = "/media/xiaohu/xiaohu/4d毫米波/view_of_delft_PUBLIC/radar/training/velodyne/00000.bin";

        // load rader points cloud
        unsigned int length = 0;
        void *data = NULL;
        std::shared_ptr<char> buffer((char *)data,std::default_delete<char[]>());
        loadData(data_file.data(), &data, &length);
        buffer.reset((char *)data);
        float *points = (float *)buffer.get();
        size_t points_size = length / sizeof(float) / 7;

		std::cout << "rader point size : " << points_size << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		float * data_ptr = static_cast<float *>(data);
		for(int i=0; i< points_size ;i++){
            pcl::PointXYZ p;
            p.x = data_ptr[7*i+0];
            p.y = data_ptr[7*i+1];
            p.z = data_ptr[7*i+2];	
			in_cloud_ptr->push_back(p);

            printf("%f %f %f %f %f %f %f \n",data_ptr[7*i+0],data_ptr[7*i+1],data_ptr[7*i+2],data_ptr[7*i+3],data_ptr[7*i+4],data_ptr[7*i+5],data_ptr[7*i+6]);
		}

    in_file.close();

    return 0;
}
