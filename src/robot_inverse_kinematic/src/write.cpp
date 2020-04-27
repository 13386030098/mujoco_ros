#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

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

            int i = 0;
            while(1)
            {
                while(pBox->flag == 0)
                {
                    i++;
                    getchar();
                    snprintf(pBox->szMsg, sizeof(pBox->szMsg), "%d", i);
                    printf("write msg is [%s]\n", pBox->szMsg);
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


































































