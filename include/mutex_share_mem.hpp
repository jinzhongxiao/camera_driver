#include <unistd.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "camera_driver/ShmInfo.h"
#include <opencv2/opencv.hpp>
#define IMG_MAX_HEIGHT 1080
#define IMG_MAX_WIDTH 1920
class ShmSubscriber{
public:
    ShmSubscriber(std::string topic_name){
        fd = -1;
        p=NULL;
    }

    cv::Mat* get_img(const camera_driver::ShmInfo::ConstPtr& data)
    {
        if(fd == -1){
             fd = shm_open(data->name.c_str(), O_RDONLY, 0666);
            if(fd < 0){
                std::cerr << "shm open failed !" << std::endl;
                return NULL;
            }     

            ftruncate(fd,sizeof(int)+data->width*data->height*3);

            p = (char*)mmap(NULL, sizeof(int)+data->width*data->height*3, PROT_READ, MAP_SHARED, fd, 0);

            if (MAP_FAILED == p) {
                perror("Error ");
                return NULL;
            }               

        }
        
        if((*(int*)p) == 1){
            img_.create(cv::Size(data->width, data->height),CV_8UC3);
            img_.data = (unsigned char*)(p + sizeof(int));
            return &img_;
        }
        else return NULL;
        
    }

    private:
        int fd = -1;
        char *p=NULL;
        cv::Mat img_;
        
};




class MutexShmObject
{
public:
    int shm_id_; 
    void *p;
    unsigned char *shm_data_;
    mutable unsigned int *written_;
 
    MutexShmObject(int datalen, int shm_key);    
    MutexShmObject(int shm_key);
 
    void write_release();
    void read_release();
    void write_ready();
    int  get_shm_id();
 
    unsigned char* get_data();
    unsigned char* get_mem();

    
 };
 
int MutexShmObject::get_shm_id(){return shm_id_;}

MutexShmObject::MutexShmObject(int shm_key)
{
    key_t key = ftok("/tmp", shm_key);
    shm_id_ = shmget(key, 0, 0);
    if(-1 == shm_id_)
    {
        perror("shm get failed");
        exit(1);
    }
    p = shmat(shm_id_, 0, 0);
    if((void*)-1 == p)
    {
        perror("shmat failed");
        exit(2);
    }
    written_ = (unsigned int*)p;
    shm_data_ = (unsigned char*)(p + sizeof(unsigned int));
    
}

unsigned char* MutexShmObject::get_mem()
{
    *written_ = 0;
    return shm_data_;
}

void MutexShmObject::write_ready(){*written_ = 1;}
unsigned char* MutexShmObject::get_data()
{
    if(*written_ == 1)
        return shm_data_;
    else 
        return NULL;
}
 
MutexShmObject::MutexShmObject(int datalen, int shm_key)
{
    key_t key = ftok("/tmp", shm_key);
  
    shm_id_ = shmget(key, datalen, IPC_CREAT|0666);//0666是文件权限，不写只有超级用户才能打开
    if(-1 == shm_id_)
    {
        perror("shmget failed");
        exit(1);
    }
    p = shmat(shm_id_, 0, 0);
    if((void*)-1 == p)
    {
        perror("shmat failed");
        exit(2);
    }
    
    written_ = (unsigned int*)p;
    shm_data_ = (unsigned char*)(p + sizeof(unsigned int));
    // initalize  share mutex
    *written_=0;
}
 

void MutexShmObject::write_release()
{
    if(-1 == shmdt(p))
    {
        perror("shmdt failed");
        exit(3);
    }

    printf("release %d\n",shm_id_);
    if(-1 == shmctl(shm_id_, IPC_RMID, NULL))
    {
        perror("shmctl failed");
        exit(4);
    }
    delete shm_data_;
}
 
void MutexShmObject::read_release()
{

    if(-1 == shmdt(p))
    {
        perror("shmdt failed");
        exit(3);
    }
    delete shm_data_;
}


