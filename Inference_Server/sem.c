#include <fcntl.h>
#include <semaphore.h>
#include <stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
sem_t* mysem[10];
void* ptr[10];
char* readbuff[10];
int count=0;
int count1=0;
char* itoa(int sum){
	if(sum>=0){
		int i=0;
		char* temp=(char*)malloc(20);
		char* msg=(char*)malloc(20);
		while(sum>0){
			temp[i]=sum%10+'0';
			sum=sum/10;
			i++;
		}
		*(msg+i+1)='\0';
		int ptr=0;
		for(int j=i; j>=1; j--){
			 *(msg+j)=temp[ptr];
			 ptr++;
		}
		*msg='0';
		free(temp);
		return msg;
	}
	else{
		sum=-sum;
		int i=0;
		char* temp=(char*)malloc(20);
		char* msg=(char*)malloc(20);
		while(sum>0){
			temp[i]=sum%10+'0';
			sum=sum/10;
			i++;
		}
		*(msg+i+1)='\0';
		int ptr=0;
		for(int j=i; j>=1; j--){
			 *(msg+j)=temp[ptr];
			 ptr++;
		}
		*msg='1';
		free(temp);
		return msg;
	}
}

int semaphore_open(char semname[], int oflag, int val){
	mysem[count]=sem_open(semname, oflag, 0666, val);
	count++;
	return count-1;
}
int getO_Creat(){
	return O_CREAT;
}

void wait(int ind){
	sem_wait(mysem[ind]);
}

void post(int ind){
	sem_post(mysem[ind]);
}

int getO_CREAT_ORDWR(){
	return O_CREAT | O_RDWR;
}

int shared_mem_open(char name[], int shm_flag){
	return  shm_open(name, shm_flag, 0666);
}

void ftrunc(int shm_fd, const int size){
	ftruncate(shm_fd, size);
}

int mmap_obj(int size, int shm_fd){
	ptr[count1]=mmap(0, size, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
	count1++;
	return count1-1;
}

void unlinkSHM(char* name){
  shm_unlink(name);
}

void writeMMF(char msg[], int mmap){
	sprintf(ptr[mmap], "%s", msg);
}

/*char* readMMF(int mmap, int size){
	char* str_read=(char*)malloc(size);
	memcpy(str_read, ptr[mmap], size);
	return str_read;
}*/
char* test(){
	char* gh=(char*)malloc(30*sizeof(char));
	strcpy(gh, "helloTest");
	return gh;
}
int main(){
	int x=shared_mem_open("hui", getO_CREAT_ORDWR());
	return 0;
}
char* readMMF(int mmap, int size){
	/*free(readbuff[mmap]);
	readbuff[mmap]=(char*)malloc(size);
	memcpy(readbuff[mmap], ptr[mmap], size);
	return readbuff[mmap];*/
 return (char*)ptr[mmap];
	//return dst;
}

void memfree(int mmap){
	free(readbuff[mmap]);
}

void mywrite(char* src, char* dst){
	int i=0;
	while(*(src+i)!='\0'){
		*(dst+i)=*(src+i);
		i++;
	}
	*(dst+i)='\0';
}

void WriteInt(int val, int mmap){
	//printf(ptr[mmap], "%d", val);
	char* arr=itoa(val);
	//printf("%d\n", val);
	//printf("%s\n", arr);
//	sprintf(ptr[mmap], "%s\n", arr);
	mywrite(arr, ptr[mmap]);
	free(arr);
	//printf("%d\n", atoi((char*)ptr[mmap]));
	//free(arr);
	//memcpy(ptr[mmap], &val, sizeof(int));
	//printf("%ls\n", (int*)ptr[mmap]);
}

int ReadInt(int mmap, int size){
	//printf("%ls\n", (int*)ptr[mmap]);
	//void* a=(int*)malloc(sizeof(int));
	//sprintf(a, "%d", *(int*)ptr[mmap]);
	//memcpy(a, ptr[mmap], sizeof(int));
	//printf("%d", *(int*)a);
	//int b=*(int*)a;
	//return b+0;
	if(*(char*)ptr[mmap]=='0')
		return atoi((char*)(ptr[mmap]+1));
	else
		return -atoi((char*)(ptr[mmap]+1));
	
	/*char* gh=(char*) malloc(50*sizeof(char));
	memcpy(gh, (char*)ptr[mmap], size);
	printf("%s", gh);
	int g=myatoi(gh);
	free(gh);
	return g;*/
}
int testme(){
	char* hi=itoa(200);
	int b= atoi(hi);
	free(hi);
	return b;
}

int getVal(int sem){
  int val;
  sem_getvalue(mysem[sem], &val);
  printf("sem value is %d and sem index is %d\n", val, sem);
  return val;
}

void reset(){
  for(int i=0; i<count; i++){
    //sem_post(mysem[i]);
    sem_destroy(mysem[i]);
  }
  count=0;
  count1=0;
}


  