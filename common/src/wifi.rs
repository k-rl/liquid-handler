use alloc::vec::Vec;
use core::result;
use deku::prelude::*;
use embassy_time::{with_timeout, Duration};
use esp_hal::rng::Rng;
use esp_radio::esp_now::{EspNow, EspNowWifiInterface, PeerInfo, BROADCAST_ADDRESS};
use thiserror::Error;

// Frame type IDs.
const SYN: u8 = 0xD0;
const SYN_ACK: u8 = 0xD1;
const DATA: u8 = 0xD2;
const ACK: u8 = 0xD3;

#[derive(Debug, Clone, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
enum Frame {
    #[deku(id = "SYN")]
    Syn(u32),
    #[deku(id = "SYN_ACK")]
    SynAck,
    #[deku(id = "DATA")]
    Data {
        seq: u32,
        #[deku(read_all)]
        payload: Vec<u8>,
    },
    #[deku(id = "ACK")]
    Ack(u32),
}

trait EspNowExt {
    fn add_mac(&mut self, mac: [u8; 6]);
    async fn recv(&mut self) -> Result<([u8; 6], Frame)>;
}

impl EspNowExt for EspNow<'_> {
    fn add_mac(&mut self, mac: [u8; 6]) {
        if !self.peer_exists(&mac) {
            self.add_peer(PeerInfo {
                interface: EspNowWifiInterface::Sta,
                peer_address: mac,
                lmk: None,
                channel: None,
                encrypt: false,
            })
            .unwrap();
        }
    }

    async fn recv(&mut self) -> Result<([u8; 6], Frame)> {
        let r = with_timeout(TIMEOUT, self.receive_async())
            .await
            .map_err(|_| Error::Timeout)?;
        let (_, frame) = Frame::from_bytes((r.data(), 0)).map_err(|_| Error::Parse)?;
        Ok((r.info.src_address, frame))
    }
}

// DATA frame overhead: id(1) + seq(4) = 5 bytes.
const MAX_PAYLOAD: usize = 250 - 5;
const TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Error, Debug)]
pub enum Error {
    #[error("Timeout error")]
    Timeout,
    #[error("Peer did not acknowledge after max retries")]
    Disconnected,
    #[error("Failed to parse frame")]
    Parse,
}

type Result<T> = result::Result<T, Error>;

#[derive(Clone, Copy)]
pub enum Role {
    Client,
    Server,
}

pub struct Socket<'a> {
    esp_now: EspNow<'a>,
    peer: [u8; 6],
    seq: u32,
}

impl<'a> Socket<'a> {
    /// Discovers a peer via ESP-NOW broadcast and establishes a connection
    /// using a TCP-like three-way handshake (SYN → SYN_ACK → ACK).
    pub async fn new(mut esp_now: EspNow<'a>, role: Role) -> Self {
        esp_now.set_channel(1).unwrap();

        let seq: u32 = Rng::new().random();

        let (peer, seq) = match role {
            Role::Client => {
                // SYN: broadcast until we get a SYN_ACK back.
                let peer = loop {
                    let _ = esp_now
                        .send_async(&BROADCAST_ADDRESS, &Frame::Syn(seq).to_bytes().unwrap())
                        .await;
                    if let Ok((mac, Frame::SynAck)) = esp_now.recv().await {
                        esp_now.add_mac(mac);
                        break mac;
                    }
                };
                (peer, seq)
            }
            Role::Server => {
                // Wait for a SYN broadcast.
                let (peer, seq) = loop {
                    if let Ok((mac, Frame::Syn(seq))) = esp_now.recv().await {
                        esp_now.add_mac(mac);
                        break (mac, seq);
                    }
                };
                let _ = esp_now
                    .send_async(&peer, &Frame::SynAck.to_bytes().unwrap())
                    .await;
                (peer, seq)
            }
        };

        Socket { esp_now, peer, seq }
    }

    /// Consumes the socket and returns the underlying `EspNow` for reuse.
    pub fn into_inner(self) -> EspNow<'a> {
        self.esp_now
    }

    pub async fn write(&mut self, data: &[u8]) -> Result<()> {
        assert!(data.len() <= MAX_PAYLOAD);

        let seq = self.seq;
        let frame = Frame::Data {
            seq,
            payload: data.to_vec(),
        }
        .to_bytes()
        .unwrap();

        for _ in 0..10 {
            let _ = self.esp_now.send_async(&self.peer, &frame).await;
            if let Ok((src, Frame::Ack(ack_seq))) = self.esp_now.recv().await {
                if src == self.peer && ack_seq == seq {
                    self.seq = self.seq.wrapping_add(1);
                    return Ok(());
                }
            }
        }

        Err(Error::Disconnected)
    }

    pub async fn read(&mut self) -> Result<Vec<u8>> {
        loop {
            let (src, frame) = self.esp_now.recv().await?;
            if src != self.peer {
                continue;
            }
            let ack_seq = match frame {
                Frame::Data { seq, payload } if seq == self.seq => {
                    self.seq = self.seq.wrapping_add(1);
                    let _ = self
                        .esp_now
                        .send_async(&self.peer, &Frame::Ack(seq).to_bytes().unwrap())
                        .await;
                    return Ok(payload);
                }
                Frame::Data { seq, .. } if seq == self.seq.wrapping_sub(1) => seq,
                _ => continue,
            };
            // Duplicate — re-ACK so the sender can proceed.
            let _ = self
                .esp_now
                .send_async(&self.peer, &Frame::Ack(ack_seq).to_bytes().unwrap())
                .await;
        }
    }
}
