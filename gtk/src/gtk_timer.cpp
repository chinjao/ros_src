#include <gtk/gtk.h>

static GdkColor colorRed;
static GdkColor colorGray;

static gint count = 10;
static guint timer_id;
GtkWidget *button1,*button2,*button3,*button4,*button5,*button6,*button7;
GtkWidget *label1;
GtkWidget *window;

gint count_down(gpointer data){
  // GtkButton *label_fi = GTK_BUTTON(button2);

}

static void cb_button1(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Happy:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
  
}

static void cb_button2(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Quiet:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

static void cb_button3(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Sad:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

static void cb_button4(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Surprise:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

static void cb_button5(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Angry:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

static void cb_button6(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Fear:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

static void cb_button7(GtkWidget *widget, gpointer user_data){
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(NULL, 
				  GTK_DIALOG_DESTROY_WITH_PARENT, 
				  GTK_MESSAGE_OTHER,
				  GTK_BUTTONS_OK,
				  "Disgust:0.321"
				  );
  gtk_dialog_run(GTK_DIALOG(dialog));
  gtk_widget_destroy(dialog);
}

int main(int argc,char **argv){

  GtkWidget *label;
  GtkWidget *vbox;
  GtkWidget *hbox;
  gulong handle;
  char buf[10];
  int i = 0;
  
  gdk_color_parse("red",&colorRed);
  gdk_color_parse("gray",&colorGray);

  gtk_init(&argc,&argv);  
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW (window), "Packing sample");
  gtk_widget_set_size_request(window,200,-1);
  /* gtk_container_set_border_width(GTK_CONTAINER(window),10);  
  label1 = gtk_label_new("Count 10");
  */
  gtk_container_add(GTK_CONTAINER(window),label1);

  g_signal_connect(G_OBJECT(window),"destroy",G_CALLBACK(gtk_main_quit),NULL);
  vbox = gtk_vbox_new(TRUE,5);
  gtk_container_add(GTK_CONTAINER(window),vbox);
  
  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
  button1 = gtk_button_new_with_label("Happy");
  gtk_box_pack_start(GTK_BOX(hbox),button1,TRUE,TRUE,0);
  gtk_widget_modify_bg(button1, GTK_STATE_SELECTED, &colorGray );  

  handle = g_signal_connect(G_OBJECT(button1),"clicked",G_CALLBACK(cb_button1),NULL);

  button2 = gtk_button_new_with_label("Quiet");
  gtk_box_pack_start(GTK_BOX(hbox),button2,TRUE,TRUE,0);
  gtk_widget_modify_bg(button2,GTK_STATE_NORMAL,&colorRed);
  gtk_widget_modify_bg(button2, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button2),"clicked",G_CALLBACK(cb_button2),NULL);


  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
 
  button3 = gtk_button_new_with_label("Sad");
  gtk_box_pack_start(GTK_BOX(hbox),button3,TRUE,TRUE,0);
  gtk_widget_modify_bg(button3, GTK_STATE_SELECTED, &colorGray );    
  handle = g_signal_connect(G_OBJECT(button3),"clicked",G_CALLBACK(cb_button3),NULL);


  button4 = gtk_button_new_with_label("Surprise");
  gtk_box_pack_start(GTK_BOX(hbox),button4,TRUE,TRUE,0);
  gtk_widget_modify_bg(button4, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button4),"clicked",G_CALLBACK(cb_button4),NULL);


  button5 = gtk_button_new_with_label("Angry");
  gtk_box_pack_start(GTK_BOX(hbox),button5,TRUE,TRUE,0);
  gtk_widget_modify_bg(button5, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button5),"clicked",G_CALLBACK(cb_button5),NULL);


  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
  button6 = gtk_button_new_with_label("Fear");
  gtk_box_pack_start(GTK_BOX(hbox),button6,TRUE,TRUE,0);
  gtk_widget_modify_bg(button6, GTK_STATE_SELECTED, &colorGray );    
  handle = g_signal_connect(G_OBJECT(button6),"clicked",G_CALLBACK(cb_button6),NULL);



  button7 = gtk_button_new_with_label("Disgust");
  gtk_box_pack_start(GTK_BOX(hbox),button7,TRUE,TRUE,0);
  gtk_widget_modify_bg(button7, GTK_STATE_SELECTED, &colorGray );  
  handle = g_signal_connect(G_OBJECT(button7),"clicked",G_CALLBACK(cb_button7),NULL);

  timer_id = g_timeout_add(1000,(GSourceFunc)count_down,NULL);



  gtk_widget_show_all(window);
  gtk_main();
  
  //  sleep(2);
  //gdk_window_clear(window->window);
   
}
