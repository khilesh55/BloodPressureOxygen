using System;
using System.Threading.Tasks;
using LiveCharts;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using Prism.Commands;
using System.Collections.Generic;
using Syncfusion.XlsIO;
using System.Windows;
using System.IO;
using System.Diagnostics;
using System.Threading;
using System.Drawing;

namespace SpasticityClient
{
    public class MeasureModel : INotifyPropertyChanged
    {
        public DateTime DateTime { get; set; }
        private double _value { get; set; }
        public double Value
        {
            get { return _value; }
            set
            {
                _value = value;
                OnPropertyChanged("Value");
            }
        }
        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged(string propertyName = null)
        {
            //Raise PropertyChanged event
            if (PropertyChanged != null)
                PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }

    public class ChartModel : INotifyPropertyChanged, IDisposable
    {
        #region variables
        private double _min;
        private double _max;
        private XBeeData _xbeeData;
        #endregion

        #region properties
        public DelegateCommand ReadCommand { get; private set; }
        public DelegateCommand SaveCommand { get; private set; }
        public DelegateCommand StopCommand { get; set; }

        public bool IsRunning { get; set; }

        public ChartValues<MeasureModel> EMGValues { get; set; }
        public ChartValues<MeasureModel> ForceValues { get; set; }
        public ChartValues<MeasureModel> AngleValues { get; set; }
        public ChartValues<MeasureModel> AngularVelocityValues { get; set; }

        public List<SessionData> SessionDatas { get; set; }

        public Func<double, string> DateTimeFormatter { get; set; }
        public double AxisStep { get; set; }
        public double AxisUnit { get; set; }

        public double Min
        {
            get { return _min; }
            set
            {
                _min = value;
                OnPropertyChanged("Min");
            }
        }
        public double Max
        {
            get { return _max; }
            set
            {
                _max = value;
                OnPropertyChanged("Max");
            }
        }

        public string PortName { get; set; }
        #endregion

        #region event
        public event PropertyChangedEventHandler PropertyChanged;
        #endregion

        #region constructor
        public ChartModel(string portname)
        {
            PortName = portname;
            _xbeeData = new XBeeData(portname);

            ReadCommand = new DelegateCommand(
                executeMethod: () =>
                {
                    Read();
                    QueryCanExecute();
                },
                canExecuteMethod: () =>
                {
                    return IsRunning == false;
                });

            SaveCommand = new DelegateCommand(
                executeMethod: () =>
                {
                    SaveData();
                    QueryCanExecute();
                },
                canExecuteMethod: () =>
                {
                    return IsRunning == false &&
                           _xbeeData.IsCancelled == true;
                });

            StopCommand = new DelegateCommand(
                executeMethod: () =>
                {
                    Dispose();
                    QueryCanExecute();
                },
                canExecuteMethod: () =>
                {
                    return IsRunning == true &&
                        _xbeeData.IsCancelled == false;
                });

            ApplicationCommands.ReadCommand.RegisterCommand(ReadCommand);
            ApplicationCommands.SaveCommand.RegisterCommand(SaveCommand);
            ApplicationCommands.StopCommand.RegisterCommand(StopCommand);

            IsRunning = false;

            //For configuring LiveCharts to use MeasureModel for X and Y
            var mapper = LiveCharts.Configurations.Mappers.Xy<MeasureModel>()
                .X(model => model.DateTime.Ticks)   //use DateTime.Ticks as X
                .Y(model => model.Value);           //use the value property as Y

            //lets save the mapper globally.
            Charting.For<MeasureModel>(mapper);

            //the values property will store our values array
            EMGValues = new ChartValues<MeasureModel>();
            ForceValues = new ChartValues<MeasureModel>();
            AngleValues = new ChartValues<MeasureModel>();
            AngularVelocityValues = new ChartValues<MeasureModel>();

            SessionDatas = new List<SessionData>();

            //lets set how to display the X Labels
            DateTimeFormatter = value => new DateTime((long)value).ToString("mm:ss");

            //AxisStep forces the distance between each separator in the X axis

            AxisStep = TimeSpan.FromSeconds(1).Ticks;

            //AxisUnit forces lets the axis know that we are plotting seconds
            //this is not always necessary, but it can prevent wrong labeling
            AxisUnit = TimeSpan.TicksPerSecond;

            SetAxisLimits(DateTime.Now);
        }
        #endregion

        #region methods

        public void Read()
        {
            try
            {
                //lets keep in memory only the last 200 records,
                //to keep everything running faster
                if (!IsRunning)
                {
                    _xbeeData.IsCancelled = false;
                    const int keepRecords = 80;
                    var mainApp = (MainWindow)App.Current.MainWindow;

                    Action readFromXBee = () =>
                    {
                        try
                        {
                            _xbeeData.Read(keepRecords, this);
                        }
                        catch (Exception ex)
                        {
                            //there is one exception, the problem is stopped, but the serial still tries to read data. It doesn't happen much, but the condition below ignores when the problem happen.
                            if (!(ex.Message == "The port is closed." && _xbeeData.IsCancelled == true))
                            {
                                //MessageBox.Show("Error reading from serial port: " + ex.Message);
                                Dispose();
                            }
                        }
                    };

                    Task.Factory.StartNew(readFromXBee);
                    _xbeeData.IsCancelled = false;
                    IsRunning = true;
                }
                else
                {
                    Dispose();
                }
            }
            catch (Exception ex)
            {
                //MessageBox.Show(ex.Message);
            }
        }

        public void SaveData()
        {
            if (!IsRunning)
            {
                using (ExcelEngine excelEngine = new ExcelEngine())
                {
                    IApplication application = excelEngine.Excel;
                    application.DefaultVersion = ExcelVersion.Excel2016;

                    //Create a workbook and enable calculations
                    IWorkbook workbook = application.Workbooks.Create(1);
                    IWorksheet worksheet = workbook.Worksheets[0];
                    worksheet.EnableSheetCalculations();

                    //Import data from SessionDatas
                    ExcelImportDataOptions importDataOptions = new ExcelImportDataOptions();
                    importDataOptions.FirstRow = 1;
                    importDataOptions.FirstColumn = 1;
                    importDataOptions.IncludeHeader = true;
                    importDataOptions.PreserveTypes = false;
                    worksheet.ImportData(SessionDatas, importDataOptions);

                    #region Calculate summary statistics and first quartile, third quartile, and interquartile range
                    //Set Labels
                    //Creating a new style with cell back color, fill pattern and font attribute
                    IStyle headingStyle = workbook.Styles.Add("NewStyle");
                    headingStyle.Color = Color.SandyBrown;
                    headingStyle.Font.Bold = true;

                    IStyle statisticStyle = workbook.Styles.Add("NewStyle2");
                    statisticStyle.Color = Color.DarkSlateBlue;
                    statisticStyle.Font.Bold = true;

                    IStyle tableBodyStyle = workbook.Styles.Add("NewStyle3");
                    tableBodyStyle.Color = Color.LightGray;

                    worksheet.Range["H1:K1"].CellStyle = headingStyle;
                    worksheet.Range["M1"].CellStyle = headingStyle;
                    worksheet.Range["G2:G11"].CellStyle = statisticStyle;
                    worksheet.Range["H2:K11"].CellStyle = tableBodyStyle;
                    worksheet.Range["M2:M11"].CellStyle = tableBodyStyle;
                    worksheet.Range["G1:K11"].CellStyle.Borders.Color = ExcelKnownColors.White;

                    worksheet.Range["M1"].Text = "Description";
                    worksheet.Range["G2"].Text = "Min";
                    worksheet.Range["M2"].Text = "Least value in the dataset";
                    worksheet.Range["G3"].Text = "Max";
                    worksheet.Range["M3"].Text = "Greatest value in the dataset";
                    worksheet.Range["G4"].Text = "Range";
                    worksheet.Range["M4"].Text = "Difference between least and greatest values";
                    worksheet.Range["G5"].Text = "Mean";
                    worksheet.Range["M5"].Text = "Average of data";
                    worksheet.Range["G6"].Text = "StDev";
                    worksheet.Range["M6"].Text = "Standard deviation of data";
                    worksheet.Range["G7"].Text = "Q1";
                    worksheet.Range["M7"].Text = "First quartile";
                    worksheet.Range["G8"].Text = "Q3";
                    worksheet.Range["M8"].Text = "Third quartile";
                    worksheet.Range["G9"].Text = "IQR";
                    worksheet.Range["M9"].Text = "Interquartile range - difference between first and third quartiles";
                    worksheet.Range["G10"].Text = "L Bound";
                    worksheet.Range["M10"].Text = "Lower bound based on 1.5x IQR";
                    worksheet.Range["G11"].Text = "U Bound";
                    worksheet.Range["M11"].Text = "Upper bound based on 1.5x IQR";
                    worksheet.Range["H1"].Text = "Angle";
                    worksheet.Range["I1"].Text = "AngVel";
                    worksheet.Range["J1"].Text = "EMG";
                    worksheet.Range["K1"].Text = "Force";

                    //Angle
                    worksheet.Range["H2"].Formula = "=MIN(B:B)";
                    worksheet.Range["H3"].Formula = "=MAX(B:B)";
                    worksheet.Range["H4"].Formula = "=ABS(H3-H2)";
                    worksheet.Range["H5"].Formula = "=AVERAGE(B:B)";
                    worksheet.Range["H6"].Formula = "=STDEV.P(B:B)";

                    worksheet.Range["H7"].Formula = "=QUARTILE(B:B,1)";
                    worksheet.Range["H8"].Formula = "=QUARTILE(B:B,3)";
                    worksheet.Range["H9"].Formula = "=ABS(H8-H7)";
                    worksheet.Range["H10"].Formula = "=H7-(H9*1.5)";
                    worksheet.Range["H11"].Formula = "=H8+(H9*1.5)";

                    //Angular Velocity
                    worksheet.Range["I2"].Formula = "=MIN(C:C)";
                    worksheet.Range["I3"].Formula = "=MAX(C:C)";
                    worksheet.Range["I4"].Formula = "=ABS(I3-I2)";
                    worksheet.Range["I5"].Formula = "=AVERAGE(C:C)";
                    worksheet.Range["I6"].Formula = "=STDEV.P(C:C)";

                    worksheet.Range["I7"].Formula = "=QUARTILE(C:C,1)";
                    worksheet.Range["I8"].Formula = "=QUARTILE(C:C,3)";
                    worksheet.Range["I9"].Formula = "=ABS(I8-I7)";
                    worksheet.Range["I10"].Formula = "=I7-(I9*1.5)";
                    worksheet.Range["I11"].Formula = "=I8+(I9*1.5)";

                    //EMG
                    worksheet.Range["J2"].Formula = "=MIN(D:D)";
                    worksheet.Range["J3"].Formula = "=MAX(D:D)";
                    worksheet.Range["J4"].Formula = "=ABS(J3-J2)";
                    worksheet.Range["J5"].Formula = "=AVERAGE(D:D)";
                    worksheet.Range["J6"].Formula = "=STDEV.P(D:D)";

                    worksheet.Range["J7"].Formula = "=QUARTILE(D:D,1)";
                    worksheet.Range["J8"].Formula = "=QUARTILE(D:D,3)";
                    worksheet.Range["J9"].Formula = "=ABS(J8-J7)";
                    worksheet.Range["J10"].Formula = "=J7-(J9*1.5)";
                    worksheet.Range["J11"].Formula = "=J8+(J9*1.5)";

                    //Force
                    worksheet.Range["K2"].Formula = "=MIN(E:E)";
                    worksheet.Range["K3"].Formula = "=MAX(E:E)";
                    worksheet.Range["K4"].Formula = "=ABS(K3-K2)";
                    worksheet.Range["K5"].Formula = "=AVERAGE(E:E)";
                    worksheet.Range["K6"].Formula = "=STDEV.P(E:E)";

                    worksheet.Range["K7"].Formula = "=QUARTILE(E:E,1)";
                    worksheet.Range["K8"].Formula = "=QUARTILE(E:E,3)";
                    worksheet.Range["K9"].Formula = "=ABS(K8-K7)";
                    worksheet.Range["K10"].Formula = "=K7-(K9*1.5)";
                    worksheet.Range["K11"].Formula = "=K8+(K9*1.5)";
                    #endregion

                    #region Highlight outliers
                    //Angle
                    //Applying conditional formatting to Angle column
                    IConditionalFormats _condition1 = worksheet.Range["B:B"].ConditionalFormats;
                    IConditionalFormat condition1 = _condition1.AddCondition();
                    condition1.FormatType = ExcelCFType.CellValue;
                    condition1.Operator = ExcelComparisonOperator.NotBetween;
                    condition1.FirstFormula = worksheet.Range["H10"].FormulaNumberValue.ToString();
                    condition1.SecondFormula = worksheet.Range["H11"].FormulaNumberValue.ToString();
                    condition1.BackColorRGB = System.Drawing.Color.FromArgb(200, 100, 100);
                    worksheet.Range["B1"].ConditionalFormats.Remove();

                    //Angular Velocity
                    //Applying conditional formatting to Angular Velocity column
                    IConditionalFormats _condition2 = worksheet.Range["C:C"].ConditionalFormats;
                    IConditionalFormat condition2 = _condition2.AddCondition();
                    condition2.FormatType = ExcelCFType.CellValue;
                    condition2.Operator = ExcelComparisonOperator.NotBetween;
                    condition2.FirstFormula = worksheet.Range["I10"].FormulaNumberValue.ToString();
                    condition2.SecondFormula = worksheet.Range["I11"].FormulaNumberValue.ToString();
                    condition2.BackColorRGB = System.Drawing.Color.FromArgb(200, 100, 100);
                    worksheet.Range["C1"].ConditionalFormats.Remove();

                    //EMG
                    //Applying conditional formatting to EMG column
                    IConditionalFormats _condition3 = worksheet.Range["D:D"].ConditionalFormats;
                    IConditionalFormat condition3 = _condition3.AddCondition();
                    condition3.FormatType = ExcelCFType.CellValue;
                    condition3.Operator = ExcelComparisonOperator.NotBetween;
                    condition3.FirstFormula = worksheet.Range["J10"].FormulaNumberValue.ToString();
                    condition3.SecondFormula = worksheet.Range["J11"].FormulaNumberValue.ToString();
                    condition3.BackColorRGB = System.Drawing.Color.FromArgb(200, 100, 100);
                    worksheet.Range["D1"].ConditionalFormats.Remove();

                    //Force
                    //Applying conditional formatting to Force column
                    IConditionalFormats _condition4 = worksheet.Range["E:E"].ConditionalFormats;
                    IConditionalFormat condition4 = _condition4.AddCondition();
                    condition4.FormatType = ExcelCFType.CellValue;
                    condition4.Operator = ExcelComparisonOperator.NotBetween;
                    condition4.FirstFormula = worksheet.Range["K10"].FormulaNumberValue.ToString();
                    condition4.SecondFormula = worksheet.Range["K11"].FormulaNumberValue.ToString();
                    condition4.BackColorRGB = System.Drawing.Color.FromArgb(200, 100, 100);
                    worksheet.Range["E1"].ConditionalFormats.Remove();

                    #endregion

                    #region Format data as table
                    //Create table with the data in given range
                    IListObject table = worksheet.ListObjects.Create("Table1", worksheet["A:E"]);
                    table.BuiltInTableStyle = TableBuiltInStyles.TableStyleMedium8;
                    #endregion

                    #region Autofit columns
                    worksheet.Range["A:E"].AutofitColumns();
                    worksheet.Range["H:K"].AutofitColumns();
                    worksheet.Range["M:M"].AutofitColumns();
                    #endregion

                    #region Charts

                    string rowcount = worksheet.UsedRange.LastRow.ToString();
                    string timeDataRange = "A2:A" + rowcount;
                    string angleDataRange = "B2:B" + rowcount;
                    string angvelDataRange = "C2:C" + rowcount;
                    string emgDataRange = "D2:D" + rowcount;
                    string forceDataRange = "E2:E" + rowcount;

                    //Add Angle Chart
                    IChartShape angleChart = worksheet.Charts.Add();
                    //Set first serie
                    IChartSerie Angle = angleChart.Series.Add("Angle");
                    Angle.Values = worksheet.Range[angleDataRange];
                    Angle.UsePrimaryAxis = true;
                    Angle.CategoryLabels = worksheet.Range[timeDataRange];
                    //Set chart details
                    angleChart.PlotArea.Border.AutoFormat = true;
                    angleChart.ChartType = ExcelChartType.Scatter_Line;
                    angleChart.HasTitle = false;
                    angleChart.IsSizeWithCell = true;
                    angleChart.Left = 584;
                    ((IChart)angleChart).Width = 836;
                    angleChart.Top = 240;
                    //Set primary value axis properties
                    angleChart.PrimaryValueAxis.Title = "Angle (°)";
                    angleChart.PrimaryCategoryAxis.Title = "Time (s)";
                    angleChart.PrimaryValueAxis.TitleArea.TextRotationAngle = -90;
                    angleChart.PrimaryCategoryAxis.HasMajorGridLines = false;
                    angleChart.PrimaryValueAxis.HasMajorGridLines = false;
                    angleChart.PrimaryValueAxis.IsAutoMax = true;
                    angleChart.PrimaryValueAxis.MinimumValue = 0;
                    //Legend position
                    angleChart.Legend.Position = ExcelLegendPosition.Bottom;
                    //View legend horizontally
                    angleChart.Legend.IsVerticalLegend = false;

                    //Add AngularVelocity Chart
                    IChartShape angvelChart = worksheet.Charts.Add();
                    //Set first serie
                    IChartSerie AngularVelocity = angvelChart.Series.Add("Angular Velocity");
                    AngularVelocity.Values = worksheet.Range[angvelDataRange];
                    AngularVelocity.UsePrimaryAxis = true;
                    AngularVelocity.CategoryLabels = worksheet.Range[timeDataRange];
                    //Set chart details
                    angvelChart.PlotArea.Border.AutoFormat = true;
                    angvelChart.ChartType = ExcelChartType.Scatter_Line;
                    angvelChart.HasTitle = false;
                    angvelChart.IsSizeWithCell = true;
                    angvelChart.Left = 584;
                    ((IChart)angvelChart).Width = 836;
                    angvelChart.Top = 640;
                    //Set primary value axis properties
                    angvelChart.PrimaryValueAxis.Title = "Angular Velocity (°/s)";
                    angvelChart.PrimaryCategoryAxis.Title = "Time (s)";
                    angvelChart.PrimaryValueAxis.TitleArea.TextRotationAngle = -90;
                    angvelChart.PrimaryCategoryAxis.HasMajorGridLines = false;
                    angvelChart.PrimaryValueAxis.HasMajorGridLines = false;
                    angvelChart.PrimaryValueAxis.IsAutoMax = true;
                    angvelChart.PrimaryValueAxis.MinimumValue = 0;
                    //Legend position
                    angvelChart.Legend.Position = ExcelLegendPosition.Bottom;
                    //View legend horizontally
                    angvelChart.Legend.IsVerticalLegend = false;

                    //Add EMG Chart
                    IChartShape emgChart = worksheet.Charts.Add();
                    //Set first serie
                    IChartSerie EMG = emgChart.Series.Add("EMG");
                    EMG.Values = worksheet.Range[emgDataRange];
                    EMG.UsePrimaryAxis = true;
                    EMG.CategoryLabels = worksheet.Range[timeDataRange];
                    //Set chart details
                    emgChart.PlotArea.Border.AutoFormat = true;
                    emgChart.ChartType = ExcelChartType.Scatter_Line;
                    emgChart.HasTitle = false;
                    emgChart.IsSizeWithCell = true;
                    emgChart.Left = 584;
                    ((IChart)emgChart).Width = 836;
                    emgChart.Top = 1040;
                    //Set primary value axis properties
                    emgChart.PrimaryValueAxis.Title = "EMG (mV)";
                    emgChart.PrimaryCategoryAxis.Title = "Time (s)";
                    emgChart.PrimaryValueAxis.TitleArea.TextRotationAngle = -90;
                    emgChart.PrimaryCategoryAxis.HasMajorGridLines = false;
                    emgChart.PrimaryValueAxis.HasMajorGridLines = false;
                    emgChart.PrimaryValueAxis.IsAutoMax = true;
                    emgChart.PrimaryValueAxis.MinimumValue = 0;
                    //Legend position
                    emgChart.Legend.Position = ExcelLegendPosition.Bottom;
                    //View legend horizontally
                    emgChart.Legend.IsVerticalLegend = false;

                    //Add Force Chart
                    IChartShape forceChart = worksheet.Charts.Add();
                    //Set first serie
                    IChartSerie Force = forceChart.Series.Add("Force");
                    Force.Values = worksheet.Range[forceDataRange];
                    Force.UsePrimaryAxis = true;
                    Force.CategoryLabels = worksheet.Range[timeDataRange];
                    //Set chart details
                    forceChart.PlotArea.Border.AutoFormat = true;
                    forceChart.ChartType = ExcelChartType.Scatter_Line;
                    forceChart.HasTitle = false;
                    forceChart.IsSizeWithCell = true;
                    forceChart.Left = 584;
                    ((IChart)forceChart).Width = 836;
                    forceChart.Top = 1440;
                    //Set primary value axis properties
                    forceChart.PrimaryValueAxis.Title = "Force (N)";
                    forceChart.PrimaryCategoryAxis.Title = "Time (s)";
                    forceChart.PrimaryValueAxis.TitleArea.TextRotationAngle = -90;
                    forceChart.PrimaryCategoryAxis.HasMajorGridLines = false;
                    forceChart.PrimaryValueAxis.HasMajorGridLines = false;
                    forceChart.PrimaryValueAxis.IsAutoMax = true;
                    forceChart.PrimaryValueAxis.MinimumValue = 0;
                    //Legend position
                    forceChart.Legend.Position = ExcelLegendPosition.Bottom;
                    //View legend horizontally
                    forceChart.Legend.IsVerticalLegend = false;
                    #endregion

                    //Set path and save
                    string spreadsheetNamePath = "acquiredData/";
                    string spreadsheetNameDate = DateTime.Now.ToString("dddd dd MMM y HHmmss");
                    string spreadsheetName = spreadsheetNamePath + spreadsheetNameDate;
                    string path = Path.GetDirectoryName(Process.GetCurrentProcess().MainModule.FileName)
                    + "\\acquiredData\\";

                    worksheet.DisableSheetCalculations();
                    workbook.SaveAs(spreadsheetName + ".xlsx");

                    #region View the Workbook
                    //Message box confirmation to view the created document.
                    if (MessageBox.Show("Do you want to view the Excel file?", "Excel file has been created",
                        MessageBoxButton.YesNo, MessageBoxImage.Information) == MessageBoxResult.Yes)
                    {
                        try
                        {
                            //Launching the Excel file using the default Application.[MS Excel Or Free ExcelViewer]
                            Process.Start(path+spreadsheetNameDate + ".xlsx");

                            //Exit
                        }
                        catch (Win32Exception ex)
                        {
                            Console.WriteLine(ex.ToString());
                        }
                    }
                    // Now that the file has been created, delete contents of SessionDatas
                    Thread.Sleep(500);
                    SessionDatas.Clear();
                    #endregion
                }
            }
        }

        public void SetAxisLimits(DateTime now)
        {
            Max = now.Ticks + TimeSpan.FromSeconds(0.3).Ticks; // lets force the axis to be 1 second ahead
            Min = now.Ticks - TimeSpan.FromSeconds(1.5).Ticks; // and 8 seconds behind
        }
        #endregion

        #region interface implementation

        void QueryCanExecute()
        {
            ReadCommand.RaiseCanExecuteChanged();
            StopCommand.RaiseCanExecuteChanged();
            SaveCommand.RaiseCanExecuteChanged();
        }

        protected virtual void OnPropertyChanged(string propertyName = null)
        {
            if (PropertyChanged != null)
                PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        protected virtual void NotifyPropertyChanged([CallerMemberName] String propertyName = "")
        {
            var handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            
            _xbeeData.IsCancelled = true;
            _xbeeData.Stop();
            EMGValues.Clear();
            ForceValues.Clear();
            AngleValues.Clear();
            AngularVelocityValues.Clear();
            IsRunning = false;
        }
        #endregion
    }
}
