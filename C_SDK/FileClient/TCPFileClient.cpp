
#include "TCPFileClient.h"

/* Safety timeout (ms) for a single file transfer. Prevents the SDK from
   blocking forever if the controller stops responding mid-transfer while the
   TCP connection stays half-open. Generous so legitimate large firmware
   uploads are not aborted; tunable. */
#define FX_FILE_TRANSFER_TIMEOUT_MS  300000L

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTCPFileClient::CTCPFileClient()
{
	m_inslen = sizeof(FileIns);
}

CTCPFileClient::~CTCPFileClient()
{

}

void CTCPFileClient::OnDisconnect()
{
	m_fop.SetErr();
	m_fop.Empty();
}

void CTCPFileClient::OnRecvData(char * data,long size)
{
	if(size != m_inslen)
	{
		printf("FX FileClient: recv frame size %ld != expected %ld (FileIns layout mismatch with controller), abort transfer\n", size, m_inslen);
		m_fop.SetErr();
		m_fop.Empty();
		return;
	}
	if( m_fop.OnIns((pFileIns)data) == true)
	{
		OnSend(data,size);
	}
}



bool CTCPFileClient::OnRecvFile(char * lpath,char * rpath)
{
	pFileIns ins = m_fop.OnRecvFile(lpath,rpath);
	if(ins == NULL)
	{
		return false;
	}
	if( OnSend((char *)ins,sizeof(FileIns)) == false)
	{
		free(ins);
		return false;
	}
	free(ins);
	long waited_ms = 0;
	while(m_fop.OnCheckStateOK() == false)
	{
		UninetSleep(50);
		waited_ms += 50;
		if (waited_ms >= FX_FILE_TRANSFER_TIMEOUT_MS)
		{
			m_fop.SetErr();
			break;
		}
	}
	
	bool err = m_fop.OnCheckErrorTag();
	m_fop.OnReSetErrorTag();
	return (err == false);
}

bool CTCPFileClient::OnSendFile(char * lpath,char * rpath)
{
	pFileIns ins = m_fop.OnSendFile(lpath,rpath);
	if(ins == NULL)
	{
		return false;
	}
	if( OnSend((char *)ins,sizeof(FileIns)) == false)
	{
		free(ins);
		return false;
	}
	free(ins);
	long waited_ms = 0;
	while(m_fop.OnCheckStateOK() == false)
	{
		UninetSleep(50);
		waited_ms += 50;
		if (waited_ms >= FX_FILE_TRANSFER_TIMEOUT_MS)
		{
			m_fop.SetErr();
			break;
		}
	}

	bool err = m_fop.OnCheckErrorTag();
	m_fop.OnReSetErrorTag();
	return (err == false);
}
