   M S B A M L   `   `   `  ���� U  QLiveCharts.Wpf, Version=0.9.7.0, Culture=neutral, PublicKeyToken=0bc1f845d1ebb8dfG4clr-namespace:LiveCharts.Wpf;assembly=LiveCharts.WpfLiveCharts.Wpf  K GSpasticityClient, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null3clr-namespace:SpasticityClientSpasticityClient !   SpasticityClient.MainWindow   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     Dlvc4clr-namespace:LiveCharts.Wpf;assembly=LiveCharts.Wpf     0localclr-namespace:SpasticityClient       � Title$  
MainWindow��$	��450��$	��800������  �� Color$ DimGray�� � DataContext * $SpasticityClient.MainWindowViewModel   � 	Resources  �� DataType!  SpasticityClient.ChartModel   EMGChart  SpasticityClient.EMGChart  
ForceChart!  SpasticityClient.ForceChart  
AngleChart!  SpasticityClient.AngleChart  AngularVelocityChart+  %SpasticityClient.AngularVelocityChart��%p   &	        &	      &	 $     &	 6     �� "  .R�  �� "  .R�  �� "  .R�  �� "  .R�  .���  � ShowGridLines .   � RowDefinitions �� $
��0.1* ��� �� �� $
��0.5* � � ColumnDefinitions �� $
��0.1* ��� �� �� $
��0.1* �.+��� $��1��$��1��#��[  ���� *
ChartModel+�� $��1��$��2��#��[ ���� *
ChartModel+�� $��2��$��1��#��[ ���� *
ChartModel+�� $��2��$��2��#��[ ���� *
ChartModel+� $��3��$��1��$��2�� �� �� �� .+��� $��0��$	��Top=�$��15q�$��Center=�$	��100��$
��Read��*  $SpasticityClient.ApplicationCommands   ReadCommand#��Z �� $��0��$��Bottom=�$��15q�$��Center=�$	��100��$
��Stop��	   StopCommand#��Z	 �� $��1��$��Center=�$��Center=�$	��150��$��Save to Spreadsheet��
   SaveCommand#��Z
 �� $��2��$��Center=�$
��Left=�$��30q�$��Port #:���� $��0��$��2��$��Right=�$��Center=�$	��100��$��30q����� *	PortNames+ �� SelectedItem �� *
PortName+