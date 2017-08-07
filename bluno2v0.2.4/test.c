/*
 * test.c
 *
 *  Created on: 2017��2��28��
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
        struct Link *head =NULL;/*ָ������ͷ*/

        printf(" do you want to append a new node (Y/N)?");
        scanf(" %c",&c);
        i = 0;
        while(c=='Y' || c=='y'){
                head = AppendNode(head);
                DispLink(head);   /*��ʾ��ǰ�����еĸ��ڵ���Ϣ*/
                printf("do you want to append a new node(Y/N)?");
                scanf(" %c",&c);
                i++;
        }
        printf("%d new nodes have been apended!\n", i);

        DeleteMemory(head);  /*�ͷ������ѷ�����ڴ�*/
}


/*
 * �������� �� �½�һ���ڵ㣬�����ýڵ���ӵ������ĩβ
 * �����Ĳ������ṹ��ָ�����head����ʾԭ�������ͷ�ڵ�ָ��
 * �����ķ���ֵ����ӽڵ��������ͷ�ڵ�ָ�� */
struct Link *AppendNode(struct Link *head){
        struct Link *p = NULL;
        struct Link *pr = head;
        int data;
        /*Ϊ����ӵĽڵ������ڴ�*/
        p = (struct Link *)malloc(sizeof(struct Link));
        if(p == NULL){/*�������ڴ�ʧ�ܣ����ӡ������Ϣ���˳�����*/
                printf("No enough memory to alloc");
                exit(0);
        }

        if(head == NULL){/*��ԭ����Ϊ�ձ����½��ڵ���Ϊ�׽ڵ�*/
                head = p;
        }else{/*��ԭ����Ϊ�ǿգ����½��ڵ���ӵ���β*/
                /*��δ����β��������ƶ�ָ��pr,ֱ��prָ���β*/
                while(pr->next != NULL){
                        pr = pr->next;
                }
                pr->next = p;/*���½��ڵ���ӵ������ĩβ*/
        }
        pr = p;/* ��prָ���½��ڵ�*/
        printf("Input node data:");
        scanf("%d",&data);/*����ڵ�����*/
        pr->data = data;
        pr->next = NULL;/*���½��ڵ���Ϊ��β*/
        return head;/*������ӽڵ��������ͷ�ڵ�ָ��*/
}



/*
 * �����Ĺ��ܣ���ʾ�����Ѿ������õĽڵ�Ľڵ�ź͸ýڵ��е�����������
 * �����Ĳ������ṹ��ָ�����head����ʾ�����ͷ�ڵ�ָ��
 * �����ķ���ֵ����*/
void DispLink(struct Link *head){
        struct Link *p = head;
        int j = 1;
        while(p != NULL){/*�����Ǳ�β����ѭ����ӡ*/
                printf("%5d%10d\n",j,p->data);/*��ӡ��j���ڵ������*/
                p = p->next;/*��pָ����һ���ڵ�*/
                j++;
        }
}


/*
 * �������ܣ��ͷ�headָ������������нڵ�ռ�õ��ڴ�
 * ����������ṹ��ָ�����head,��ʾ�����ͷ�ڵ�ָ��
 * ���ز�������*/
void DeleteMemory(struct Link *head){
        struct Link *p= head, *pr = NULL;
        while(p != NULL){/*�����Ǳ�β�����ͷŽڵ�ռ�õ��ڴ�*/
                pr = p;/*��pr�б��浱ǰ�ڵ��ָ��*/
                p = p->next;/*��pָ����һ�ڵ�*/
                free(pr);/*�ͷ�prָ��ĵ�ǰ�ڵ�ռ�õ��ڴ�*/

        }
}


/*
 * �������ܣ���headָ���������ɾ��һ���ڵ�����ΪnodeData�Ľڵ�
 * ����������ṹ��ָ�����head,��ʾԭ�������ͷ�ڵ�ָ�����ͱ���nodeData,��ʾ��ɾ���ڵ�Ľڵ�����ֵ
 * ���ز�����ɾ���ڵ��������ͷ�ڵ�ָ��*/

struct Link *DelNode(struct Link *head,int nodeData){

        struct Link *p = head, *pr = head;
        if(head == NULL){/*����Ϊ�գ�û�нڵ㣬�޷�ɾ��*/
                printf("No Linken Table!\n");
                return(head);
        }
        /*��û�ҵ��ڵ�nodeData��δ����β,�������*/
        while(nodeData != p->data && p->next != NULL){
                pr = p;
                p = p->next;
        }
        if(nodeData == p->data){/*���ҵ��ڵ�nodeData,��ɾ���ýڵ�*/
                if(p == head){/*����ɾ���ڵ�Ϊ�׽ڵ㣬����headָ���2���ڵ�*/
                        head = p->next;
                }else{/*��ɾ���ڵ㲻���׽ڵ㣬��ǰһ�ڵ��ָ��ָ��ǰ�ڵ����һ�ڵ�*/
                        pr->next = p->next;
                }
                free(p);
        }else{
                printf("This Node has not been found!\n");
        }
        return head;/*����ɾ���ڵ��������ͷ�ڵ�ָ��*/
}



#endif

