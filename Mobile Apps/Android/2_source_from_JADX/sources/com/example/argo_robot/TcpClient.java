package com.example.argo_robot;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Toast;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;

public class TcpClient {
    public static final int SERVER_PORT = 8001;
    public static final String TAG = TcpClient.class.getSimpleName();
    public static String mServerMessage;
    boolean connection_status = false;
    private BufferedReader mBufferIn;
    /* access modifiers changed from: private */
    public PrintWriter mBufferOut;
    private OnMessageReceived mMessageListener = null;
    private boolean mRun = false;
    MainActivity main;
    Socket socket;

    public interface OnMessageReceived {
        void messageReceived(String str);
    }

    public TcpClient(OnMessageReceived listener) {
        this.mMessageListener = listener;
    }

    public void sendMessage(final String message) {
        new Thread(new Runnable() {
            public void run() {
                if (TcpClient.this.mBufferOut != null) {
                    String str = TcpClient.TAG;
                    Log.d(str, "Sending: " + message);
                    TcpClient.this.mBufferOut.println(message);
                    TcpClient.this.mBufferOut.flush();
                }
            }
        }).start();
    }

    public void stopClient() {
        this.mRun = false;
        PrintWriter printWriter = this.mBufferOut;
        if (printWriter != null) {
            printWriter.flush();
            this.mBufferOut.close();
        }
        this.mMessageListener = null;
        this.mBufferIn = null;
        this.mBufferOut = null;
        mServerMessage = null;
    }

    public void run(final Context conn, String SERVER_IP) {
        Socket socket2;
        this.mRun = true;
        try {
            InetAddress serverAddr = InetAddress.getByName(SERVER_IP);
            Log.d("TCP Client", "C: Connecting...");
            try {
                this.socket = new Socket(serverAddr, SERVER_PORT);
                this.connection_status = true;
            } catch (Exception e) {
                this.connection_status = false;
            }
            try {
                this.mBufferOut = new PrintWriter(new BufferedWriter(new OutputStreamWriter(this.socket.getOutputStream())), true);
                this.mBufferIn = new BufferedReader(new InputStreamReader(this.socket.getInputStream()));
                while (this.mRun) {
                    String readLine = this.mBufferIn.readLine();
                    mServerMessage = readLine;
                    if (!(readLine == null || this.mMessageListener == null)) {
                        this.mMessageListener.messageReceived(readLine);
                    }
                }
                Log.d("RESPONSE FROM SERVER", "S: Received Message: '" + mServerMessage + "'");
                socket2 = this.socket;
            } catch (Exception e2) {
                Log.e("TCP", "S: Error", e2);
                new Handler(Looper.getMainLooper()).post(new Runnable() {
                    public void run() {
                        Toast.makeText(conn, "Oops! No route to host", 0).show();
                    }
                });
                socket2 = this.socket;
            }
            socket2.close();
        } catch (Exception e3) {
            Log.e("TCP", "C: Error", e3);
            new Handler(Looper.getMainLooper()).post(new Runnable() {
                public void run() {
                    Toast.makeText(conn, "Oops! Connection Refused from the Server", 0).show();
                }
            });
        } catch (Throwable th) {
            this.socket.close();
            throw th;
        }
    }

    public String getName() {
        return mServerMessage;
    }

    public String get_variables() {
        return String.valueOf(this.connection_status);
    }
}
