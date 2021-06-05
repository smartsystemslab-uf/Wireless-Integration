package com.example.argo_robot;

import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.internal.view.SupportMenu;
import com.example.argo_robot.TcpClient;

public class MainActivity extends AppCompatActivity {
    String Rec_String;
    Button back;
    Button connect;
    Boolean connect_flag = false;
    Button front;

    /* renamed from: ip */
    EditText f60ip;
    String ip_text;
    Button left;
    TcpClient mTcpClient;
    String message_to_be_sent;
    Button right;
    Button stop;
    Toast tst;

    /* access modifiers changed from: protected */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView((int) C0296R.layout.activity_main);
        this.connect = (Button) findViewById(C0296R.C0298id.connect);
        this.front = (Button) findViewById(C0296R.C0298id.front);
        this.back = (Button) findViewById(C0296R.C0298id.back);
        this.left = (Button) findViewById(C0296R.C0298id.left);
        this.right = (Button) findViewById(C0296R.C0298id.right);
        this.stop = (Button) findViewById(C0296R.C0298id.stop);
        this.f60ip = (EditText) findViewById(C0296R.C0298id.f61ip);
        this.connect.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                int action = arg1.getAction();
                if (action != 0) {
                    return action == 1;
                }
                if (!MainActivity.this.connect_flag.booleanValue()) {
                    MainActivity mainActivity = MainActivity.this;
                    mainActivity.ip_text = mainActivity.f60ip.getText().toString();
                    new ConnectTask().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, new String[0]);
                } else if (MainActivity.this.connect_flag.booleanValue()) {
                    MainActivity.this.connect_flag = false;
                    if (MainActivity.this.mTcpClient != null) {
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        MainActivity.this.mTcpClient.stopClient();
                        MainActivity.this.connect.setText("connect");
                        MainActivity.this.connect.setBackgroundColor(-16711936);
                    }
                }
                return true;
            }
        });
        this.front.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                if (arg1.getAction() == 0) {
                    MainActivity.this.message_to_be_sent = "Front";
                    new send_data().execute(new String[]{"1"});
                }
                return true;
            }
        });
        this.back.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                if (arg1.getAction() == 0) {
                    MainActivity.this.message_to_be_sent = "Back";
                    new send_data().execute(new String[]{"1"});
                }
                return true;
            }
        });
        this.left.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                if (arg1.getAction() == 0) {
                    MainActivity.this.message_to_be_sent = "Left";
                    new send_data().execute(new String[]{"1"});
                }
                return true;
            }
        });
        this.right.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                if (arg1.getAction() == 0) {
                    MainActivity.this.message_to_be_sent = "Right";
                    new send_data().execute(new String[]{"1"});
                }
                return true;
            }
        });
        this.stop.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View arg0, MotionEvent arg1) {
                if (arg1.getAction() == 0) {
                    MainActivity.this.message_to_be_sent = "Stop";
                    new send_data().execute(new String[]{"1"});
                }
                return true;
            }
        });
    }

    public void Connection() {
        new ConnectTask().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, new String[0]);
    }

    public class ConnectTask extends AsyncTask<String, String, TcpClient> {
        public ConnectTask() {
        }

        /* access modifiers changed from: protected */
        public TcpClient doInBackground(String... message) {
            MainActivity.this.mTcpClient = new TcpClient(new TcpClient.OnMessageReceived() {
                public void messageReceived(String message) {
                    MainActivity.this.Rec_String = message;
                    Log.d("Pressed", "recieved: " + MainActivity.this.Rec_String);
                    ConnectTask.this.publishProgress(new String[]{message});
                }
            });
            try {
                MainActivity.this.mTcpClient.run(MainActivity.this, MainActivity.this.ip_text);
                Log.d("Pressed", "connection request sent");
                return null;
            } catch (Exception e) {
                Log.d("Pressed", "error in connection");
                return null;
            }
        }

        /* access modifiers changed from: protected */
        public void onProgressUpdate(String... values) {
            super.onProgressUpdate(values);
            if (values[0].contains("Thank you")) {
                MainActivity.this.connect_flag = true;
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                MainActivity.this.toasts("connected");
                MainActivity.this.connect.setBackgroundColor(SupportMenu.CATEGORY_MASK);
                MainActivity.this.connect.setText("Disconnect");
            }
        }
    }

    public void toasts(String string) {
        try {
            this.tst.cancel();
        } catch (Exception e) {
        }
        Toast makeText = Toast.makeText(this, string, 0);
        this.tst = makeText;
        makeText.show();
    }

    public class send_data extends AsyncTask<String, String, Integer> {
        public send_data() {
        }

        /* access modifiers changed from: protected */
        public Integer doInBackground(String... message) {
            if (MainActivity.this.mTcpClient != null) {
                Log.d("Pressed", "sending: " + String.valueOf(MainActivity.this.message_to_be_sent));
                MainActivity.this.mTcpClient.sendMessage(MainActivity.this.message_to_be_sent);
                MainActivity.this.runOnUiThread(new Runnable() {
                    public void run() {
                        MainActivity.this.toasts(String.valueOf(MainActivity.this.message_to_be_sent));
                        MainActivity.this.message_to_be_sent = "";
                    }
                });
            } else {
                Log.d("Pressed", String.valueOf("not connected"));
                MainActivity.this.runOnUiThread(new Runnable() {
                    public void run() {
                        MainActivity.this.toasts("Could not connect to controller");
                    }
                });
            }
            return 0;
        }
    }
}
