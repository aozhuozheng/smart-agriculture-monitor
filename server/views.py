from django.shortcuts import render
from pyecharts.charts import Line, Gauge
from pyecharts import options as opts
from .models import TempHumiData
import requests


# 生成温湿度折线图
def get_line_chart():
    # 读取数据库最新20条数据
    data_list = TempHumiData.objects.all().order_by("-create_time")[:20]
    time_x = [str(data.create_time.strftime("%H:%M:%S")) for data in data_list]
    temp_y = [data.temp for data in data_list]
    humi_y = [data.humi for data in data_list]
    line = (
        Line(init_opts=opts.InitOpts(width="1000px", height="400px"))
        .add_xaxis(time_x)
        .add_yaxis("温度℃", temp_y, markpoint_opts=opts.MarkPointOpts(data=[opts.MarkPointItem(type_="max"), opts.MarkPointItem(type_="min")]))
        .add_yaxis("湿度%RH", humi_y)
        .set_global_opts(
            title_opts=opts.TitleOpts(title="实时温湿度时序曲线"),
            xaxis_opts=opts.AxisOpts(axislabel_opts=opts.LabelOpts(rotate=30)),
            datazoom_opts=opts.DataZoomOpts()
        )
    )
    return line.render_embed()


# 生成温度仪表盘
def get_gauge(temp_val):
    gauge = (
        Gauge(init_opts=opts.InitOpts(width="400px", height="400px"))
        .add("温度", [("℃", temp_val)], min_=0, max_=50)
        .set_global_opts(title_opts=opts.TitleOpts(title="当前温度仪表盘"))
    )
    return gauge.render_embed()


# 监控主页视图
def index(request):
    # 获取最新一条传感器数据
    latest_data = TempHumiData.objects.last()
    line_html = get_line_chart()
    gauge_html = get_gauge(latest_data.temp if latest_data else 25)
    return render(request, "index.html", {"line": line_html, "gauge": gauge_html})


# 从OneNET云平台拉取传感器数据并存入数据库
def pull_onenet_data():
    headers = {"Authorization": "token 你的设备Token"}
    url = "https://api.heclouds.com/devices/设备ID/datapoints"
    res = requests.get(url, headers=headers)
    res_json = res.json()
    # 解析云端温湿度
    temp = res_json["data"]["datastreams"][0]["datapoints"][0]["value"]
    humi = res_json["data"]["datastreams"][1]["datapoints"][0]["value"]
    # 存入MySQL
    TempHumiData.objects.create(temp=float(temp), humi=float(humi))
