/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include "UDPClient.h"

using namespace smartlink;


UDPClient::UDPClient() {
    bzero(&remoteAddr, sizeof(remoteAddr));
    remoteAddr.sin_family = AF_INET;
}

UDPClient::~UDPClient() {
}

void UDPClient::initParam(char *ip_r, int port) {
    remoteAddr.sin_port = htons( port );
    inet_pton( AF_INET, ip_r, &remoteAddr.sin_addr );
}

int UDPClient::connect() {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    return 0;
}

int UDPClient::close() {
    ::close(fd);
    return 0;
}

int UDPClient::recMsg(char *buf, int len_r) {
    struct sockaddr_in recvAddr;
    int len = sizeof(recvAddr);
    bzero(&recvAddr, sizeof(recvAddr));
    int res = recvfrom(fd, buf, len_r, 0, (struct sockaddr *) &recvAddr,
                       (socklen_t * ) & len);

    return res;
}

int UDPClient::sendMsg(char *buf, int len_r) {
    return sendto(fd, buf, len_r, 0, (const sockaddr *) &remoteAddr,
                  sizeof(remoteAddr));
}
