#include "udp_client.h"

udp_client::udp_client(string address,int port)
{
    /* 建立socket，注意必须是SOCK_DGRAM */
    if ((sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket");
    }
    /* 填写sockaddr_in*/
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr=inet_addr(address.c_str());
    addr.sin_port = htons(port);
//    addr.sin_addr.s_addr = inet_addr(address.c_str());
    while(1)
    {
        if(connect(sockfd,(struct sockaddr*)& addr,sizeof(addr))==-1)
        {
            cout<<"connect() error"<<endl;
            delay(100);
            continue;
        }
        else
        {
            break;
        }
    }
    cout<<"connected...."<<endl;
}


string udp_client::start_listen()
{

    char buffer[1024];
    int len = 0;
    int addr_len = sizeof(struct sockaddr_in);

    bzero(buffer, sizeof(buffer));

    buffer[0]=robot_id;
    len=1;
//        len = read(STDIN_FILENO, buffer, sizeof(buffer));

    write(sockfd,buffer,strlen(buffer));
    len=read(sockfd,buffer,sizeof(buffer));
//    int recv_len=0;
//    while(recv_len<256)
//    {
//        recv_cnt=read(sockfd,&buffer[recv_l])
//    }

//    /* 将字符串传送给server端*/
//    sendto(sockfd, buffer, len, 0, (struct sockaddr *)&addr, addr_len);
//    /* 接收server端返回的字符串*/

//    timeval tv={0,200000};
//    setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));

//    len = recvfrom(sockfd, buffer, sizeof(buffer), 0,(struct sockaddr *)&addr, (socklen_t *)&addr_len);

    string markers_pos="";
    if(len!=-1)
    {
        markers_pos=buffer;
    }
    return markers_pos;
}
