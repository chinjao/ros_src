#include <gtk/gtk.h>

static void cb_button_clicked(GtkButton *widget,gpointer user_data){
  static int count = 0;
  char buf[1024];
  sprintf(buf,"%d time clicked", ++count);
  gtk_button_set_label(widget,buf);

}



int main(int argc,char **argv){
  GtkWidget *window;
  GtkWidget *vbox;
  GtkWidget *hbox;
  GtkWidget *button;
  GtkWidget *canvas;
  int i = 0;
  char buf[100];
  gtk_init(&argc,&argv);  
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW (window), "Packing sample");
  gtk_container_set_border_width(GTK_CONTAINER(window),200);
  
  g_signal_connect(G_OBJECT(window),"destroy",G_CALLBACK(gtk_main_quit),NULL);
  vbox = gtk_vbox_new(TRUE,5);
  gtk_container_add(GTK_CONTAINER(window),vbox);
  
  hbox = gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,TRUE,TRUE,0);
  button = gtk_button_new_with_label("click me");
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(cb_button_clicked),NULL);
  sprintf(buf,"%d",i);
  button = gtk_button_new_with_label(buf);
  gtk_box_pack_start(GTK_BOX(hbox),button,TRUE,TRUE,0);
    

  gtk_widget_show_all(window);
  gtk_main();
  
  sleep(2);
  //gdk_window_clear(window->window);
  i++;
  
}
