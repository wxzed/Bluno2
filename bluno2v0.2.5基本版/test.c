/*
 * test.c
 *
 *  Created on: 2017年2月28日
 *      Author: Administrator
 */
#if 0
#include <stdio.h>
#include <stdlib.h>

struct Link *AppendNode(struct Link *head);
static void DispLink(struct Link *head);
static void DeleteMemory(struct Link *head);
static void wx_main( void);

struct Link{
        int data;
        struct Link *next;
};

wx_main(){
        int i;
        char c;
        struct Link *head =NULL;/*指向链表头*/

        printf(" do you want to append a new node (Y/N)?");
        scanf(" %c",&c);
        i = 0;
        while(c=='Y' || c=='y'){
                head = AppendNode(head);
                DispLink(head);   /*显示当前链表中的各节点信息*/
                printf("do you want to append a new node(Y/N)?");
                scanf(" %c",&c);
                i++;
        }
        printf("%d new nodes have been apended!\n", i);

        DeleteMemory(head);  /*释放所有已分配的内存*/
}


/*
 * 函数功能 ： 新建一个节点，并将该节点添加到链表的末尾
 * 函数的参数：结构体指针变量head，表示原有链表的头节点指针
 * 函数的返回值：添加节点后的链表的头节点指针 */
struct Link *AppendNode(struct Link *head){
        struct Link *p = NULL;
        struct Link *pr = head;
        int data;
        /*为新添加的节点申请内存*/
        p = (struct Link *)malloc(sizeof(struct Link));
        if(p == NULL){/*若申请内存失败，则打印错误信息，退出程序*/
                printf("No enough memory to alloc");
                exit(0);
        }

        if(head == NULL){/*若原链表为空表，则将新建节点置为首节点*/
                head = p;
        }else{/*若原链表为非空，则将新建节点添加到表尾*/
                /*若未到表尾，则继续移动指针pr,直到pr指向表尾*/
                while(pr->next != NULL){
                        pr = pr->next;
                }
                pr->next = p;/*将新建节点添加到链表的末尾*/
        }
        pr = p;/* 让pr指向新建节点*/
        printf("Input node data:");
        scanf("%d",&data);/*输入节点数据*/
        pr->data = data;
        pr->next = NULL;/*将新建节点置为表尾*/
        return head;/*返回添加节点后的链表的头节点指针*/
}



/*
 * 函数的功能：显示所有已经建立好的节点的节点号和该节点中的数据项内容
 * 函数的参数：结构体指针变量head，表示链表的头节点指针
 * 函数的返回值：无*/
void DispLink(struct Link *head){
        struct Link *p = head;
        int j = 1;
        while(p != NULL){/*若不是表尾，则循环打印*/
                printf("%5d%10d\n",j,p->data);/*打印第j个节点的数据*/
                p = p->next;/*让p指向下一个节点*/
                j++;
        }
}


/*
 * 函数功能：释放head指向的链表中所有节点占用的内存
 * 输入参数：结构体指针变量head,表示链表的头节点指针
 * 返回参数：无*/
void DeleteMemory(struct Link *head){
        struct Link *p= head, *pr = NULL;
        while(p != NULL){/*若不是表尾，则释放节点占用的内存*/
                pr = p;/*在pr中保存当前节点的指针*/
                p = p->next;/*让p指向下一节点*/
                free(pr);/*释放pr指向的当前节点占用的内存*/

        }
}


/*
 * 函数功能：从head指向的链表中删除一个节点数据为nodeData的节点
 * 输入参数：结构体指针变量head,表示原有链表的头节点指针整型变量nodeData,表示待删除节点的节点数据值
 * 返回参数：删除节点后的链表的头节点指针*/

struct Link *DelNode(struct Link *head,int nodeData){

        struct Link *p = head, *pr = head;
        if(head == NULL){/*链表为空，没有节点，无法删除*/
                printf("No Linken Table!\n");
                return(head);
        }
        /*若没找到节点nodeData且未到表尾,则继续找*/
        while(nodeData != p->data && p->next != NULL){
                pr = p;
                p = p->next;
        }
        if(nodeData == p->data){/*若找到节点nodeData,则删除该节点*/
                if(p == head){/*若待删除节点为首节点，则让head指向第2个节点*/
                        head = p->next;
                }else{/*若删除节点不是首节点，则将前一节点的指针指向当前节点的下一节点*/
                        pr->next = p->next;
                }
                free(p);
        }else{
                printf("This Node has not been found!\n");
        }
        return head;/*返回删除节点后的链表的头节点指针*/
}



#endif

