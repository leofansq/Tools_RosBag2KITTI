/*  This code is to transform the .pcd files to .bin files.
*
*   The point cloud files decoded from the .bag file are usually in .pcd format. 
* In order to facilitate the experiment of 3D detection, the format of the KITTI 
* data set needs to be unified, that is, converted into the .bin format.In the 
* .bin file, each point corresponds to four data, which are xyz and intensity.
* 
*   Input file: a .pcd file containing xyz and intensity
*   Output file: a .bin file in KITTI format
*   Parameters to be adjusted: file path in the Main function
*
*   Code by FSQ 2018.11.5
**************************************************************************************/

#include <iostream>           
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>     
using namespace std;

//Transform PCD 2 BIN
void pcd2bin (string &in_file, string& out_file)
{ 
   //Create a PointCloud value
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  //Open the PCD file
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (in_file, *cloud) == -1) 
  {
    PCL_ERROR ("Couldn't read in_file\n");
  }
  //Create & write .bin file
  ofstream bin_file(out_file.c_str(),ios::out|ios::binary|ios::app);
  if(!bin_file.good()) cout<<"Couldn't open "<<out_file<<endl;  

  //PCD 2 BIN 
  cout << "Converting "
            << in_file <<"  to  "<< out_file
            << endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
  	bin_file.write((char*)&cloud->points[i].x,3*sizeof(float)); 
    bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
    //cout<< 	cloud->points[i]<<endl;
  }
  	
  bin_file.close();
}
static std::vector<std::string> file_lists;

//Read the file lists
void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

//Sort the file list
void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

int main(int argc, char **argv)
{
    //Set the file path
    std::string bin_path = "/home/cecilia/leo_projects/bishe2019/pcd2bin/bin/";
    std::string pcd_path = "/home/cecilia/leo_projects/bishe2019/pcd2bin/pcd/";
    //Read file lists of specific type
    read_filelists( pcd_path, file_lists, "pcd" );
    sort_filelists( file_lists, "pcd" );
    //PCD2BIN one by one
    for (int i = 0; i < file_lists.size(); ++i)
    {
        std::string pcd_file = pcd_path + file_lists[i];
        std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".bin";
        std::string bin_file = bin_path + tmp_str;
        pcd2bin( pcd_file, bin_file);
    }

    return 0;
}
