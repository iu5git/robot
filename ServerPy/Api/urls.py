from django.contrib import admin
from django.urls import path, include
from Api import views
# V  DRF  V
from rest_framework import routers

router = routers.DefaultRouter()
router.register(r'api/test', views.TestApiView, basename='TestApiViewModel')
# router.register(r'api/login', views.LoginApiView, basename='ApiLoginModel')
# router.register(r'api/logout', views.LoginApiView, basename='ApiLoginModel1')
# router.register(r'api/connect', views.LoginApiView, basename='ApiLoginModel2')
# router.register(r'api/command', views.LoginApiView, basename='ApiLoginModel3')
# router.register(r'api/robo-connect', views.LoginApiView, basename='ApiLoginModel4')

urlpatterns = [
  # path('api/login', views.login_page_router),
  # path('api/logout', views.login_page_router),
  # path('api/connect', views.login_page_router),
  # path('api/command', views.login_page_router),
  # path('api/robo-connect', views.login_page_router),

  # V  API  V
  path('', include(router.urls)),
  path('api/', include('rest_framework.urls', namespace='rest_framework'))
]
