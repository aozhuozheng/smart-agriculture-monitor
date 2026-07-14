from django.urls import path
from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("pull_data/", views.pull_onenet_data, name="pull"),
]
