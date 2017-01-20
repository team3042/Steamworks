package org.usfirst.frc.team3042.robot.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;

import org.spectrum3847.RIOdroid.RIOadb;
import org.usfirst.frc.team3042.robot.Robot;
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
    private volatile boolean wantsAppRestart = false;
    private Socket p;

    private ArrayList<ServerThread> serverThreads = new ArrayList<>();
    private ArrayList<VisionUpdateReceiver> receivers = new ArrayList<>();

    VisionUpdate mostRecentUpdate = null;

    private static final int port = 3042;

    public static VisionServer getInstance() {
        if (instance == null) {
            instance = new VisionServer();
        }
        return instance;
    }

    private boolean connected = false;

    public boolean isConnected() {
        return connected;
    }

    // Thread to deal with sending heartbeat messages and receiving heartbeat
    // and vision messages
    protected class ServerThread implements Runnable {
        private Socket socket;

        public ServerThread(Socket socket) {
            this.socket = socket;
        }

        // Taking a message, converting it to json ending with a newline, and
        // sending
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

        // Takes a message, sends it back if heartbeat, sends to recievers
        // otherwise
        public void handleMessage(VisionMessage message, double timestamp) {
            if ("targets".equals(message.getType())) {
                System.out.println("Received vision message");
                VisionUpdate update = VisionUpdate.generateFromJsonString(timestamp, message.getMessage());
                receivers.removeAll(Collections.singleton(null));
                if (update.isValid()) {
                    System.out.println("Vision update valid");
                    for (VisionUpdateReceiver receiver : receivers) {
                        System.out.println("Sent to receiver");
                        receiver.gotUpdate(update);
                    }
                }

                mostRecentUpdate = update;
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
                socket = null;
                p = null;
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
        System.out.println("VisionServer initializing");
        try {
            // Creating a socket and setting up connection over adb to start tcp
            serverSocket = new ServerSocket(port);
            RIOadb.init();
            AdbUtils.adbReverseForward(port, port);
        } catch (IOException e) {
            e.printStackTrace();
        }
        new Thread(this).start();
        new Thread(new AppMaintainanceThread()).start();
    }

    /**
     * If a VisionUpdate object (i.e. a target) is not in the list, add it.
     * 
     * @see VisionUpdate
     */
    public void addVisionUpdateReceiver(VisionUpdateReceiver receiver) {
        if (!receivers.contains(receiver)) {
            receivers.add(receiver);
        }
    }

    public void removeVisionUpdateReceiver(VisionUpdateReceiver receiver) {
        if (receivers.contains(receiver)) {
            receivers.remove(receiver);
        }
    }

    public VisionUpdate getMostRecentUpdate() {
        if (mostRecentUpdate == null) {
            System.out.println("No updates available");
        }

        return mostRecentUpdate;
    }

    @Override
    public void run() {
        System.out.println("VisionServer thread starting");

        p = null;
        while (running) {
            // Creating new threads to communicate with server every 100 ms,
            // which self-terminate after no messages are available
            try {
                if (p == null) {
                    System.out.println("Attempting to accept socket");
                    p = serverSocket.accept();
                    System.out.println("Socket Accepted!");
                }
                ServerThread s = new ServerThread(p);
                new Thread(s).start();
                serverThreads.add(s);
            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private class AppMaintainanceThread implements Runnable {
        @Override
        public void run() {
            while (true) {
                if (getTimestamp() - lastMessageReceivedTime > .1) {
                    // camera disconnected
                    AdbUtils.adbReverseForward(port, port);
                    connected = false;
                } else {
                    connected = true;
                }
                if (wantsAppRestart) {
                    AdbUtils.restartApp();
                    wantsAppRestart = false;
                }
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private double getTimestamp() {
        return System.currentTimeMillis();
    }

}
