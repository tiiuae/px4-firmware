use core::fmt;
use embedded_io::ErrorKind;
use crate::nuttx::galloc::alloc::vec::Vec;

extern "C" {
    fn socket(domain: i32, ty: i32, protocol: i32) -> i32;
    fn bind(sockfd: i32, addr: *const u8, addrlen: u32) -> i32;
    fn sendto(sockfd: i32, buf: *const u8, len: u64, flags: i32, dest_addr: *const u8, addrlen: u32) -> i32;
    fn close(sockfd: i32);
}

/// A UDP socket.

pub struct UdpSocket {
    fd: i32,
}

impl Drop for UdpSocket {
    fn drop(&mut self) {
        unsafe {
            if self.fd >= 0 {
                close(self.fd);
                self.fd = -1;
            }
        }
    }
}

impl UdpSocket {
    const AF_INET: u16 = 2;
    const SOCK_DGRAM: i32 = 2;

    fn do_bind(&mut self, addr: &str) -> Result<(), ErrorKind> {
        let fd = unsafe { socket(UdpSocket::AF_INET as i32, UdpSocket::SOCK_DGRAM, 0) };
        if fd < 0 {
            Err(ErrorKind::Other)
        } else {
            self.fd = fd;
            let socketaddr = UdpSocket::get_socketaddr(addr);
            let ret = unsafe { bind(fd, socketaddr.as_ptr(), 16) };
            if ret < 0 {
                return Err(ErrorKind::Other);
            }
            Ok(())
        }
    }

    // a example: '192.168.200.100:12345'"
    fn get_socketaddr(a: &str) -> [u8; 16] {
        let addr_port: Vec<&str> = a.split(":").collect();
        let octets: Vec<&str> = addr_port[0].split(".").collect();
        let mut socketaddr = [0; 16];

        let dom = UdpSocket::AF_INET.to_le_bytes();   // LE for host
        let port = addr_port[1].parse::<u16>().unwrap().to_be_bytes();        // BE for network
        let addr: Vec<u8> = octets.iter().map(|x| x.parse::<u8>().unwrap()).collect::<Vec<u8>>().as_slice().try_into().unwrap();

        socketaddr[0] = dom[0];
        socketaddr[1] = dom[1];
        socketaddr[2] = port[0];
        socketaddr[3] = port[1];
        socketaddr[4] = addr[0];
        socketaddr[5] = addr[1];
        socketaddr[6] = addr[2];
        socketaddr[7] = addr[3];
        socketaddr
    }

    pub fn bind(addr: &str) -> Result<UdpSocket, ErrorKind> {
        let mut socket = UdpSocket{fd: -1};
        socket.do_bind(&addr)?;
        return Ok(socket);
    }

/*
    pub fn recv_from(&self, buf: &mut [u8]) -> io::Result<(usize, ErrorKind)> {
        // TODO: impl
    }
*/

    pub fn send_to(&self, buf: &[u8], addr: &str) -> Result<usize, ErrorKind> {
        let socketaddr = UdpSocket::get_socketaddr(addr);
        let ret = unsafe { sendto(self.fd, buf.as_ptr(), buf.len() as u64, 0, socketaddr.as_ptr(), 16) };
        if ret < 0 {
            return Err(ErrorKind::Other);
        }
        Ok(ret as usize)
    }
}


impl fmt::Debug for UdpSocket {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // TODO: print socket info
        f.debug_struct("UdpSocket").finish()
    }
}
