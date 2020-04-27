#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
            while(1)
            {
                if(pBox->flag == 1)
                {
                        printf("msg from writer is [%s]\n", pBox->szMsg);
                        pBox->flag = 0;
                }
            }
            shmdt(shm);
            if(0 == shmctl(shm_id, IPC_RMID, 0))
            {
                printf("delete shm success.\n");
            }

        }else{
             perror("shmat:");
        }
    }else{
        perror("shmget:");
    }
    return 0;
}



































