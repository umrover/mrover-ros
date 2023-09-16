from django.urls import path, include
from notes import views

urlpatterns = [
    path("chat/<str:chat_box_name>/", views.chat_box, name="chat"),
]
