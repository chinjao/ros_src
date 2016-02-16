#include <gtk/gtk.h>
#include <iostream>
#include <string>

int main(int argc,char **argv){
  GtkWidget *window;
  int i = 0;
  char buf[100];
  gtk_init(&argc,&argv);
  
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_show  (window);
    
  gtk_main ();

  return 0;
}
