// 定义消息常量
private static final int UPDATE_DATA = 0x001;
private static final int UPDATE_FAN_STATUS = 0x002;

// 主线程Handler
private Handler mHandler = new Handler(Looper.getMainLooper()){
    @Override
    public void handleMessage(@NonNull Message msg) {
        super.handleMessage(msg);
        switch (msg.what){
            case UPDATE_DATA:
                // 更新温湿度UI
                String[] data = (String[]) msg.obj;
                tvTemp.setText("当前温度：" + data[0] + " ℃");
                tvHumi.setText("当前湿度：" + data[1] + " %RH");
                break;
            case UPDATE_FAN_STATUS:
                // 更新风扇动画状态
                boolean isOpen = (boolean) msg.obj;
                updateFanAnimation(isOpen);
                break;
        }
    }
};

// 云端数据循环请求线程
private void startNetThread(){
    new Thread(new Runnable() {
        @Override
        public void run() {
            while (!isDestroy){
                try {
                    // 1. 配置OneNET请求参数
                    String url = "https://api.heclouds.com/devices/你的设备ID/datapoints";
                    OkHttpClient client = new OkHttpClient();
                    Request request = new Request.Builder()
                            .url(url)
                            .addHeader("Authorization", "token 你的设备Token")
                            .build();
                    Response response = client.newCall(request).execute();
                    String jsonStr = response.body().string();

                    // 2. JSON数据解析
                    JSONObject jsonObject = new JSONObject(jsonStr);
                    JSONArray dataArray = jsonObject.getJSONObject("data").getJSONArray("datastreams");
                    
                    // 解析温度、湿度、风扇状态
                    String temp = dataArray.getJSONObject(0).getJSONArray("datapoints").getJSONObject(0).getString("value");
                    String humi = dataArray.getJSONObject(1).getJSONArray("datapoints").getJSONObject(0).getString("value");
                    boolean fanState = "1".equals(dataArray.getJSONObject(2).getJSONArray("datapoints").getJSONObject(0).getString("value"));

                    // 3. 发送消息更新UI
                    mHandler.obtainMessage(UPDATE_DATA, new String[]{temp, humi}).sendToTarget();
                    mHandler.obtainMessage(UPDATE_FAN_STATUS, fanState).sendToTarget();

                } catch (Exception e) {
                    e.printStackTrace();
                }
                try {
                    Thread.sleep(1000); // 1秒刷新一次
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }).start();
}
public class DataService extends Service {
    @Override
    public void onCreate() {
        super.onCreate();
        // 启动网络数据请求线程
        startNetThread();
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    // 后台持续运行，监控网络状态
    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }
}


// 网络状态广播接收器
private class NetBroadcast extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        // 网络断开自动重连
        if(!isNetworkAvailable()){
            Toast.makeText(MainActivity.this, "网络断开，正在重连...", Toast.LENGTH_SHORT).show();
        }
    }
}
