#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>

#define TEST_SIZE 2048

typedef struct _BOX
{
        int  flag;
        char szMsg[TEST_SIZE];
}Box;


int main()
{
    int shm_id = shmget(13, 2048, IPC_CREAT | 0666);
    if (shm_id != -1)
    {
        void* shm = shmat(shm_id, NULL, 0);
        if(shm != (void*)-1)
        {
            Box *pBox = (Box*)shm;
            pBox->flag = 0;

            auto s1 = std::to_string(42.5);
            std::string s2 = " ";
            auto s3 = std::to_string(50.9);
            std::string str = s1 + s2 +s3;
            char* p = (char*)str.data();
             while(1)
            {
                while(pBox->flag == 0)
                {
                    getchar();
                    snprintf(pBox->szMsg, sizeof(pBox->szMsg), "%s", p);
//                    memcpy(shm, str, str.size() + 1);
                    printf("shm = %s\n", pBox->szMsg);
                    pBox->flag = 1;
                }

            }
            shmdt(shm);

        }else{
             perror("shmat:");
        }
    }else{
        perror("shmget:");
    }
    return 0;
}


































































