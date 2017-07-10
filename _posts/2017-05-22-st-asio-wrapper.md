---
title: st-asio-wrapper
date : 2017-05-22 16:00:00
---

# st_asio_wrapper
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 简介
·`st_asio_wrapper` 是基于 `Boost.Asio` 的异步 `C/S` 通信框架，因为项目中使用到这个库，所以这里记录下使用的方法。

这里是它的 GitHub 地址：[st_asio_wrapper ](https://github.com/youngwolf-project/st_asio_wrapper)

具体的用法在它的网站上已经有了详细的介绍，这里总结下自己的使用方法和踩过的坑。

## TCP-TCP 连接

#### TCP 客户端
为了实现 `TCP` 的连接，我们在客户端需要继承 `st_connector`，然后重写 `on_msg_handle` 或者 `on_msg` 函数来自定义处理消息，核心代码如下所示：
```cpp
#pragma once

//configuration
#undef MAX_MSG_NUM
#define MAX_MSG_NUM	   1500
//#define FORCE_TO_USE_MSG_RECV_BUFFER  //force to use the msg recv buffer

#include "stdafx.h" 

#include "communication/st_asio_wrapper_base.h"
#include "communication/st_asio_wrapper_tcp_client.h"
#include <string>
using namespace st_asio_wrapper;

class CTcpClient
{
public:
	CTcpClient();
	~CTcpClient();

private:
	class MyConnector : public st_connector
	{
		public:
			MyConnector(boost::asio::io_service& io_service_) : st_connector(io_service_){}
		protected:
			typedef std::string MsgType;
            // 自定义处理消息
			virtual void on_msg_handle(MsgType& msg)
			{
				AfxMessageBox(_T("on_msg_handle"));
			}
			
			virtual bool on_msg(MsgType& msg)
			{
				AfxMessageBox(_T("on_msg"));
				return true;
			}
	};

private: 
	st_service_pump  m_pump;
	st_sclient<MyConnector> m_client;
};
```
这里只是部分的核心代码，全部的代码最后会给出链接。

#### TCP 服务器端
我们在 TCP 服务器端直接发送指定的消息即可，核心代码如下所示：
```cpp
class CTcpServer
{
public:
	CTcpServer();
	~CTcpServer();

private:
	typedef std::string msg_type;

public: 
    // 发送一条字符串消息
	void send_msg(const msg_type& p_msg)
    { 
	    m_server.broadcast_msg(p_msg);  
    }

private: 
	class My_st_server_socket : public st_server_socket_base<>
	{
		public:
			typedef std::string msg_type;
		public:
			My_st_server_socket(i_server& server_): st_server_socket_base(server_){}  
	};

private: 
	st_service_pump m_pump;
	st_server_base<My_st_server_socket> m_server;   
};
```


## UDP - UDP 链接
#### UDP 发送
`UDP` 比 `TCP` 要简单些，因为 `UDP` 是面向无连接的协议。下面 是`UDP` 发送的核心代码：
```cpp
#pragma once
#include "communication/st_asio_wrapper_base.h"
#include "communication/st_asio_wrapper_udp_client.h"
#include "communication/st_asio_wrapper_udp_socket.h"
#include "communication/st_asio_wrapper_client.h"
using namespace st_asio_wrapper;

class CUdpServer
{
public:
	CUdpServer();
	~CUdpServer();
public:
	typedef std::string MsgType;
    // 发送 UDP 消息
	void SendMsg(const MsgType& msg)
    {
        m_UdpServer.safe_send_native_msg(m_PeerAddr, msg);
    }
private:
	boost::asio::ip::udp::endpoint m_PeerAddr; 
	st_service_pump m_UdpPump;
	st_udp_sclient m_UdpServer;
};
```
#### UDP 接收
我们在接收 `UDP` 消息的时候，需要继承 `st_udp_socket` 类，重写 `on_msg_handle` 或者 `on_msg` 来自定义消息处理。下面是 `UDP` 接收的核心代码：
```cpp
#pragma once

#include "communication/st_asio_wrapper_base.h"
#include "communication/st_asio_wrapper_udp_socket.h"
#include "communication/st_asio_wrapper_udp_client.h"
#include "communication/st_asio_wrapper_client.h"

using namespace st_asio_wrapper;

class CUdpClient
{
public:
	CUdpClient();
	~CUdpClient();
private:
	class MyUdpConnector : public st_udp_socket
	{
		typedef std::string msg_type;

		public:
			MyUdpConnector(boost::asio::io_service& io_service_): st_udp_socket(io_service_) {}
		protected:
            // 重写 on_msg 
			virtual bool on_msg(msg_type& msg)
			{
				//自定义处理消息
				AfxMessageBox(_T("Test"));
				return true;
			}
            // 重写 on_msg_handle
			virtual bool on_msg_handle(msg_type& msg)
			{
				return true;
			}
	};

private:
	st_service_pump m_Pump;
	st_sclient<MyUdpConnector> m_client;
	boost::asio::ip::udp::endpoint m_PeerAddr;
};
```

## 注意
在 TCP 的客户端和服务器端和 UDP 发送以及接收端，你要**保证你的通信库是相同的版本**，否则可能出现接收不到消息的情况发送，我之前的遇到过这种情况，因为自己下载过比较新的版本，而项目使用的是老的版本，结果导致连接失败。
