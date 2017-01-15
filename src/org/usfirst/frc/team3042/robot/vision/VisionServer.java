package org.usfirst.frc.team3042.robot.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;

import org.spectrum3847.RIOdroid.RIOadb;
import org.usfirst.frc.team3042.robot.vision.messages.HeartbeatMessage;
import org.usfirst.frc.team3042.robot.vision.messages.OffWireMessage;
import org.usfirst.frc.team3042.robot.vision.messages.VisionMessage;

import edu.wpi.first.wpilibj.Timer;

// Code taken from team 254 github

public class VisionServer implements Runnable {
	
	private static VisionServer instance = null;
	private ServerSocket serverSocket;
	double lastMessageReceivedTime = 0;
	private boolean running = true;
	
	private ArrayList<ServerThread> serverThreads = new ArrayList<>();
    private ArrayList<VisionUpdateReceiver> receivers = new ArrayList<>();
	
	private static final int port = 3042;
	
	public static VisionServer getInstance() {
		if(instance == null) {
			instance = new VisionServer();
		}
		return instance;
	}
	
	// Thread to deal with sending heartbeat messages and receiving heartbeat and vision messages
	protected class ServerThread implements Runnable {
		private Socket socket;
		
		public ServerThread(Socket socket) {
			this.socket = socket;
		}
		
		// Taking a message, converting it to json ending with a newline, and sending
		public void send(VisionMessage message) {
            String toSend = message.toJson() + "\n";
            if (socket != null && socket.isConnected()) {
                try {
                    OutputStream os = socket.getOutputStream();
                    os.write(toSend.getBytes());
                } catch (IOException e) {
                    System.err.println("VisionServer: Could not send data to socket");
                }
            }
        }
		
		public void handleMessage(VisionMessage message, double timestamp) {
            if ("targets".equals(message.getType())) {
                VisionUpdate update = VisionUpdate.generateFromJsonString(timestamp, message.getMessage());
                receivers.removeAll(Collections.singleton(null));
                if (update.isValid()) {
                    for (VisionUpdateReceiver receiver : receivers) {
                        receiver.gotUpdate(update);
                    }
                }
            }
            if ("heartbeat".equals(message.getType())) {
                send(HeartbeatMessage.getInstance());
            }
        }
		
		public boolean isAlive() {
            return socket != null && socket.isConnected() && !socket.isClosed();
        }
		
		@Override
		public void run() {
			// Once thread used, do not run entire loop
			if (socket == null) {
                return;
            }
			try {
				InputStream is = socket.getInputStream();
                byte[] buffer = new byte[2048];
                int read;
                // Continue while connected and have messages to read
                while (socket.isConnected() && (read = is.read(buffer)) != -1) {
                	double timestamp = getTimestamp();
                	lastMessageReceivedTime = timestamp;
                    String messageRaw = new String(buffer, 0, read);
                    
                    // Process messages split by newlines
                    String[] messages = messageRaw.split("\n");
                    for (String message : messages) {
                    	OffWireMessage parsedMessage = new OffWireMessage(message);
                        if (parsedMessage.isValid()) {
                            handleMessage(parsedMessage, timestamp);
                        }
                    }
                }
                System.out.println("Socket disconnected");
			} catch (IOException e) {
                System.err.println("Could not talk to socket");
            }
			if (socket != null) {
                try {
                    socket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
		}
	}
	
	private VisionServer() {
		try {
			// Creating a socket and setting up connection over adb to start tcp
			serverSocket = new ServerSocket(port);
			RIOadb.init();
			AdbUtils.adbReverseForward(port, port);
		} catch (IOException e) {
			e.printStackTrace();
		}
		new Thread(this).start();
	}
	
	@Override
	public void run() {
		while(running) {
			// Creating new threads to communicate with server every 100 ms, which self-terminate after no messages are available
			try {
                Socket p = serverSocket.accept();
                ServerThread s = new ServerThread(p);
                new Thread(s).start();
                serverThreads.add(s);
            } catch (IOException e) {
                System.err.println("Issue accepting socket connection!");
            } finally {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
		}
	}
	
	private double getTimestamp() {
		return Timer.getFPGATimestamp();
	}

}
