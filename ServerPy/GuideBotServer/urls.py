import imp
from django.contrib import admin
from django.urls import path, include
from GuideBotServer.views import login_page_router

urlpatterns = [
  path('admin/', admin.site.urls),
  path('test', login_page_router),
  path('', include('WebClient.urls')),
  path('', include('Api.urls')),
]
