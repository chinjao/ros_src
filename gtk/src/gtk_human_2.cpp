#include <gtk/gtk.h>
#include <pthread.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech_msgs/Bayesian.h"

#include <map>

#define TIMFONT "IPA Gothic 20"

using namespace std;

static GdkColor colorRed;
static GdkColor colorGray;
static GdkColor colorWhite;

static gint count = 10;
static guint timer_id;


map< string,map <string ,float > > emotion_trans;
map< string,map<string,float > >  cur_emo_value;
map< string,int> name_o;
float pos,net,nor;
string cur_emo;
string name;

GtkWidget *label1;
GtkWidget *window_main;
GtkWidget *vbox_main;
GtkWidget *hbox_main;

pthread_mutex_t mutex;

gint count_down(gpointer data){
}

static void cb_button(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  GtkButton *Button = GTK_BUTTON(widget);
  const gchar *text = gtk_button_get_label(Button); 
  char str[100];
  sprintf(str,"%s:0.321",text);
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "%s",
				  str
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
  
}


static void human_button(GtkButton *button_na, gpointer user_data){
  GtkWidget *vbox;
  GtkWidget *hbox;
  gulong handle;
  GtkWidget *window;
  GtkWidget *button;
  
  const gchar *text = gtk_button_get_label(button_na); 
  char str[100];
  sprintf(str,"%s Emotion",text);
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW (window), str);
  gtk_widget_set_size_request(window,300,150);

  g_signal_connect(G_OBJECT(window),"destroy",G_CALLBACK(gtk_main_quit),NULL);
  vbox = gtk_vbox_new(TRUE,5);
  gtk_container_add(GTK_CONTAINER(window),vbox);
  
  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
  button = gtk_button_new_with_label("Happy");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );  

  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);

  button = gtk_button_new_with_label("Quiet");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button,GTK_STATE_NORMAL,&colorRed);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);


  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
 
  button = gtk_button_new_with_label("Sad");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );    
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);


  button = gtk_button_new_with_label("Surprise");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);


  button = gtk_button_new_with_label("Angry");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);


  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
  button = gtk_button_new_with_label("Fear");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );    
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);



  button = gtk_button_new_with_label("Disgust");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button),NULL);

  timer_id = g_timeout_add(1000,(GSourceFunc)count_down,NULL);
  g_signal_connect (window, "delete_event", G_CALLBACK (gtk_window_iconify), NULL);

  gtk_widget_show_all(window);
 
}

void* thread1(void* pParam){
  string name;
  float pos,neg,nor;
  sleep(1);
  const char *str;
  GtkWidget *button;
  gulong handle;
  GtkTextBuffer *buffer;  
  GtkWidget *label;
  GtkWidget *top_event_box;


  while(1){
    cout << "name:";
    cin >> name;
    cout << endl;
    str = name.c_str();
    hbox_main = gtk_hbox_new(FALSE,0);
    gtk_box_pack_start(GTK_BOX(vbox_main),hbox_main,TRUE,TRUE,0);
    button = gtk_button_new_with_label(str);
    gtk_box_pack_start(GTK_BOX(hbox_main),button,TRUE,TRUE,0);
    gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray ); 

    top_event_box = gtk_event_box_new ();
    gtk_widget_modify_bg (top_event_box, GTK_STATE_NORMAL, &colorGray);
    gtk_box_pack_start (GTK_BOX (hbox_main), top_event_box, TRUE, TRUE, 2);
    {
      GtkWidget *top_label;
      label = gtk_label_new ("Quiet");
      gtk_widget_modify_fg (label, GTK_STATE_NORMAL, &colorWhite);
      gtk_container_add (GTK_CONTAINER (top_event_box), label);
    }

    handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(human_button),NULL);
    gtk_widget_show_all(window_main);

  }
}

void* listener(void* pParam){
  ros::NodeHandle nh;
  while(1){
    speech_msgs::BayesianConstPtr bayes_msg = ros::topic::waitForMessage<speech_msgs::Bayesian>("bayes_result");
  }
}

int main(int argc,char **argv){

  ros::init(argc,argv,"gtk_visualize");
  GtkWidget *button;
  gulong handle;
  GtkTextBuffer *buffer;
  pthread_t tid1,tid2;
  GtkWidget *label;
  GtkWidget *top_event_box;
  PangoFontDescription *font_description;

  pthread_mutex_init(&mutex,NULL);

  pthread_create(&tid1,NULL,thread1,NULL);
  
  pthread_create(&tid2,NULL,listener,NULL);

  gdk_color_parse("red",&colorRed);
  gdk_color_parse("gray",&colorGray);
  gdk_color_parse("#FFFFFF", &colorWhite);

  gtk_init(&argc,&argv);
  window_main = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW (window_main), "Member");
  gtk_widget_set_size_request(window_main,300,-1);

  g_signal_connect(G_OBJECT(window_main),"destroy",G_CALLBACK(gtk_main_quit),NULL);
  vbox_main = gtk_vbox_new(TRUE,5);
  gtk_container_add(GTK_CONTAINER(window_main),vbox_main);
  
  hbox_main = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox_main),hbox_main,TRUE,TRUE,0);
  button = gtk_button_new_with_label("Hirai");
  gtk_widget_set_size_request(button,150,50);

  font_description = pango_font_description_from_string(TIMFONT);
  gtk_widget_modify_font(GTK_WIDGET(button), font_description);
  gtk_box_pack_start(GTK_BOX(hbox_main),button,TRUE,TRUE,0);
  gtk_widget_modify_bg(button, GTK_STATE_SELECTED, &colorGray );
  
  top_event_box = gtk_event_box_new ();
  gtk_widget_modify_bg (top_event_box, GTK_STATE_NORMAL, &colorGray);
  gtk_box_pack_start (GTK_BOX (hbox_main), top_event_box, TRUE, TRUE, 2);
  {
    GtkWidget *top_label;
    label = gtk_label_new ("Happy");
    gtk_widget_modify_fg (label, GTK_STATE_NORMAL, &colorWhite);
    gtk_container_add (GTK_CONTAINER (top_event_box), label);
  }

  
  handle = g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(human_button),NULL);

  gtk_widget_show_all(window_main);

  gtk_main();
}
