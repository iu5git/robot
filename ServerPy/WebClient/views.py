from urllib import response
import logging
from django.http import HttpResponse, FileResponse
from django.shortcuts import render
from django.conf import settings

# https://www.youtube.com/watch?v=nAK-Tpc3NMI
logger = logging.getLogger(__name__)
# logger = logging.getLogger("common")

def index_page_router(request):
  # из /templates
  # return render(request, 'index.html')
  logger.info("index.html")
  return FileResponse(open(settings.STATIC_ROOT + '/index.html', 'rb'))
