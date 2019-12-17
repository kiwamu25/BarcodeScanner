using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Sensors;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Graphics.Display;
using Windows.Graphics.Imaging;
using Windows.Media;
using Windows.Media.Capture;
using Windows.Media.MediaProperties;
using Windows.Storage;
using Windows.Storage.FileProperties;
using Windows.Storage.Streams;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using ZXing;
using static ZXing.RGBLuminanceSource;
// 空白ページの項目テンプレートについては、https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x411 を参照してください

namespace BarcodeScanner
{
    /// <summary>
    /// それ自体で使用できる空白ページまたはフレーム内に移動できる空白ページ。
    /// </summary>
    public sealed partial class MainPage : Page
    {
        ViewModel viewModel = new ViewModel();
        MediaCapture mediaCapture;
        DispatcherTimer timer1;
        private bool _externalCamera;
        private bool _mirroringPreview;
        DeviceInformation _cameraDevice;
        private CameraRotationHelper _rotationHelper;

        bool Trycheck = false;
        public MainPage()
        {
            this.InitializeComponent();
            timer1 = new DispatcherTimer();
            timer1.Interval = new TimeSpan(300);
            timer1.Tick += Timer1_Tick;
            MediaCaptureInitializeAsync();
        }


        public async void Dispose()
        {
            if (mediaCapture != null)
            {
                await mediaCapture.StopPreviewAsync();
                mediaCapture.Dispose();
                mediaCapture = null;
                _cameraDevice = null;
                if (timer1 != null)
                {
                    timer1.Stop();
                    timer1 = null;
                }
            }
            GC.Collect();
        }

        public async static Task<SoftwareBitmap> SoftwareBitmapRotate(SoftwareBitmap softwarebitmap, uint width, uint height, BitmapRotation Rotation)
        {
            using (InMemoryRandomAccessStream stream = new InMemoryRandomAccessStream())
            {
                BitmapEncoder encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.BmpEncoderId, stream);
                encoder.SetSoftwareBitmap(softwarebitmap);
                encoder.BitmapTransform.Rotation = Rotation;
                await encoder.FlushAsync();

                BitmapDecoder decoder = await BitmapDecoder.CreateAsync(stream);

                return await decoder.GetSoftwareBitmapAsync(softwarebitmap.BitmapPixelFormat, BitmapAlphaMode.Premultiplied);
            }
        }

        public async static Task<SoftwareBitmap> GetCroppedBitmapAsync(SoftwareBitmap softwareBitmap, uint startPointX, uint startPointY, uint width, uint height)
        {
            using (InMemoryRandomAccessStream stream = new InMemoryRandomAccessStream())
            {
                BitmapEncoder encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.BmpEncoderId, stream);

                encoder.SetSoftwareBitmap(softwareBitmap);

                encoder.BitmapTransform.Bounds = new BitmapBounds()
                {
                    X = startPointX,
                    Y = startPointY,
                    Height = height,
                    Width = width
                };


                await encoder.FlushAsync();

                BitmapDecoder decoder = await BitmapDecoder.CreateAsync(stream);

                return await decoder.GetSoftwareBitmapAsync(softwareBitmap.BitmapPixelFormat, BitmapAlphaMode.Premultiplied);
            }
        }

        private async Task TryDecodePreviewAsync()
        {
            if (!Trycheck)
            {
                Trycheck = true;
               // await mediaCapture.VideoDeviceController.FocusControl.FocusAsync();
                // Get information about the preview
                var previewProperties = mediaCapture.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview) as VideoEncodingProperties;
                if (previewProperties == null)
                    return;
                // このVideoFrameのFormatとBarcodeReaderのフォーマットで一致するのがGray8しかない
                var videoFrame = new VideoFrame(BitmapPixelFormat.Bgra8, (int)previewProperties.Width, (int)previewProperties.Height);
                // Capture the preview frame
                using (var currentFrame = await mediaCapture.GetPreviewFrameAsync(videoFrame))
                {
                    // 結果フレームを取得
                    var previewFrame = currentFrame.SoftwareBitmap;

                    var dispInfo = DisplayInformation.GetForCurrentView();
                    var dispRotate = BitmapRotation.None;

                    uint ScanAreaWidth, ScanAreaHeight;
                    uint CenterX, CenterY;
                    uint ScanHeight = 350;
                    switch (dispInfo.CurrentOrientation)
                    {

                        case DisplayOrientations.Portrait:
                        case DisplayOrientations.PortraitFlipped:
                            ScanAreaWidth = ScanHeight;
                            ScanAreaHeight = (uint)previewFrame.PixelHeight;
                            CenterX = (uint)((previewFrame.PixelWidth / 2) - (ScanHeight / 2));
                            CenterY = 0;
                            dispRotate = BitmapRotation.Clockwise90Degrees;
                            break;
                        default:
                            ScanAreaWidth = (uint)previewFrame.PixelWidth;
                            ScanAreaHeight = ScanHeight;
                            CenterX = 0;
                            CenterY = (uint)((previewFrame.PixelHeight / 2) - (ScanHeight / 2));
                            break;
                    }


                    viewModel.ScanHeight = (PreviewControl.ActualHeight / previewFrame.PixelHeight * ScanHeight);
                    var SoftBitMap = await GetCroppedBitmapAsync(previewFrame, CenterX, CenterY, ScanAreaWidth, ScanAreaHeight);

                    //画像回転
                    SoftBitMap = await SoftwareBitmapRotate(SoftBitMap, ScanAreaWidth, ScanAreaHeight, dispRotate);

                    //var SoftBitMap = await GetCroppedBitmapAsync(previewFrame, 0, 0, (uint)previewFrame.PixelWidth, (uint)previewFrame.PixelHeight);
                    SoftBitMap = SoftwareBitmap.Convert(SoftBitMap, BitmapPixelFormat.Gray8);


                    // 結果をbyte配列に変換。DecodeメソッドはWriteableBitmapも受け付けるが、
                    // こちらはUIスレッド上でないと生成できないのであきらめる。

                    var buffer = new byte[4 * SoftBitMap.PixelWidth * SoftBitMap.PixelHeight];
                    SoftBitMap.CopyToBuffer(buffer.AsBuffer());
                    //閾値計算　明るさの最大値から固定値割
                    var MaxBrite = 0;
                    var thr = 0;
                    var Range = viewModel.Range; //明るさ最大値 / Range
                    foreach (var v in buffer)
                    {
                        if (MaxBrite < v)
                            MaxBrite = v;
                    }
                    thr = (int)(MaxBrite * Range);
                    //求めた閾値から二値化
                    for (int x = 0; x < buffer.Length; x++)
                    {
                        if (buffer[x] < thr)
                            buffer[x] = 0;
                        else
                            buffer[x] = 255;
                    }
                    viewModel.Threshold = thr;
                    viewModel.MaxBrite = MaxBrite;
                    //変換後の画像表示デバッグ用
                    IBuffer bufferI = buffer.AsBuffer();
                    var sampleBitmap = SoftwareBitmap.CreateCopyFromBuffer(bufferI, BitmapPixelFormat.Gray8, SoftBitMap.PixelWidth, SoftBitMap.PixelHeight);
                    sampleBitmap = SoftwareBitmap.Convert(sampleBitmap, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Premultiplied);
                    await viewModel.ScanImage.SetBitmapAsync(sampleBitmap);
                    var barcodeReader = new BarcodeReader { AutoRotate = true };
                    //PossibleFormatsで読み取るコードを設定できる
                    //barcodeReader.Options.PossibleFormats = new[] { BarcodeFormat.CODE_128 }.ToList();
                    var r = barcodeReader.Decode(buffer, SoftBitMap.PixelWidth, SoftBitMap.PixelHeight, BitmapFormat.Gray8);
                    Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                    {
                        var ResultText = r?.Text ?? "";
                        if (ResultText != string.Empty)
                        {
                            viewModel.ResultText = ResultText;
                        }
                    }).FireAndForget();
                }

                Trycheck = false;
            }
        }

        private async void MediaCaptureInitializeAsync()
        {
            //DisplayInformation.AutoRotationPreferences = DisplayOrientations.Landscape;
            var allVideoDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);
            DeviceInformation desiredDevice = allVideoDevices.FirstOrDefault(x => x.EnclosureLocation != null
                && x.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Back);

            _cameraDevice = desiredDevice ?? allVideoDevices.FirstOrDefault();


            if (_cameraDevice == null)
            {
                System.Diagnostics.Debug.WriteLine("No camera device found!");
                return;
            }

            var settings = new MediaCaptureInitializationSettings { VideoDeviceId = _cameraDevice.Id };

            mediaCapture = new MediaCapture();
            mediaCapture.RecordLimitationExceeded += MediaCapture_RecordLimitationExceeded;
            mediaCapture.Failed += MediaCapture_Failed;

            try
            {
                await mediaCapture.InitializeAsync(settings);
            }
            catch (UnauthorizedAccessException)
            {
                System.Diagnostics.Debug.WriteLine("The app was denied access to the camera");
                return;
            }

            // Handle camera device location
            if (_cameraDevice.EnclosureLocation == null ||
                _cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Unknown)
            {
                _externalCamera = true;
            }
            else
            {
                _externalCamera = false;
                _mirroringPreview = (_cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
            }
            _rotationHelper = new CameraRotationHelper(_cameraDevice.EnclosureLocation);
            _rotationHelper.OrientationChanged += RotationHelper_OrientationChanged;
            PreviewControl.Source = mediaCapture;
            PreviewControl.FlowDirection = _mirroringPreview ? FlowDirection.RightToLeft : FlowDirection.LeftToRight;
            await mediaCapture.StartPreviewAsync();
            await SetPreviewRotationAsync();

            timer1.Start();
        }


        private void MediaCapture_FocusChanged(MediaCapture sender, MediaCaptureFocusChangedEventArgs args)
        {
        }

        private async void Timer1_Tick(object sender, object e)
        {
            try
            {
                if (timer1.IsEnabled)
                {
                    await TryDecodePreviewAsync();
                }
            }
            catch { }
        }

        private void MediaCapture_Failed(MediaCapture sender, MediaCaptureFailedEventArgs errorEventArgs)
        {
        }

        private void MediaCapture_RecordLimitationExceeded(MediaCapture sender)
        {
        }

        private async Task SetPreviewRotationAsync()
        {
            if (!_externalCamera)
            {
                // Add rotation metadata to the preview stream to make sure the aspect ratio / dimensions match when rendering and getting preview frames
                var rotation = _rotationHelper.GetCameraPreviewOrientation();
                var props = mediaCapture.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview);
                Guid RotationKey = new Guid("C380465D-2271-428C-9B83-ECEA3B4A85C1");
                props.Properties.Add(RotationKey, CameraRotationHelper.ConvertSimpleOrientationToClockwiseDegrees(rotation));
                await mediaCapture.SetEncodingPropertiesAsync(MediaStreamType.VideoPreview, props, null);
            }
        }

        private async void RotationHelper_OrientationChanged(object sender, bool updatePreview)
        {
            if (updatePreview)
            {
                await SetPreviewRotationAsync();
            }
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                // Rotate the buttons in the UI to match the rotation of the device
                var angle = CameraRotationHelper.ConvertSimpleOrientationToClockwiseDegrees(_rotationHelper.GetUIOrientation());
                var transform = new RotateTransform { Angle = angle };

                // The RenderTransform is safe to use (i.e. it won't cause layout issues) in this case, because these buttons have a 1:1 aspect ratio

            });
        }


        private async Task CapturePhotoWithOrientationAsync()
        {
            var captureStream = new InMemoryRandomAccessStream();
            try
            {
                await mediaCapture.CapturePhotoToStreamAsync(ImageEncodingProperties.CreateJpeg(), captureStream);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("Exception when taking a photo: {0}", ex.ToString());
                return;
            }

            var decoder = await BitmapDecoder.CreateAsync(captureStream);
            var file = await KnownFolders.PicturesLibrary.CreateFileAsync("SimplePhoto.jpeg", CreationCollisionOption.GenerateUniqueName);

            using (var outputStream = await file.OpenAsync(FileAccessMode.ReadWrite))
            {
                var encoder = await BitmapEncoder.CreateForTranscodingAsync(outputStream, decoder);
                var photoOrientation = CameraRotationHelper.ConvertSimpleOrientationToPhotoOrientation(
                    _rotationHelper.GetCameraCaptureOrientation());
                var properties = new BitmapPropertySet {
            { "System.Photo.Orientation", new BitmapTypedValue(photoOrientation, PropertyType.UInt16) } };
                await encoder.BitmapProperties.SetPropertiesAsync(properties);
                await encoder.FlushAsync();
            }
        }

        private async Task StartRecordingWithOrientationAsync()
        {
            try
            {
                var videoFile = await KnownFolders.VideosLibrary.CreateFileAsync("SimpleVideo.mp4", CreationCollisionOption.GenerateUniqueName);

                var encodingProfile = MediaEncodingProfile.CreateMp4(VideoEncodingQuality.Auto);

                var rotationAngle = CameraRotationHelper.ConvertSimpleOrientationToClockwiseDegrees(
                    _rotationHelper.GetCameraCaptureOrientation());
                Guid RotationKey = new Guid("C380465D-2271-428C-9B83-ECEA3B4A85C1");
                encodingProfile.Video.Properties.Add(RotationKey, PropertyValue.CreateInt32(rotationAngle));

                await mediaCapture.StartRecordToStorageFileAsync(encodingProfile, videoFile);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("Exception when starting video recording: {0}", ex.ToString());
            }
        }

        class CameraRotationHelper
        {
            private EnclosureLocation _cameraEnclosureLocation;
            private DisplayInformation _displayInformation = DisplayInformation.GetForCurrentView();
            private SimpleOrientationSensor _orientationSensor = SimpleOrientationSensor.GetDefault();
            public event EventHandler<bool> OrientationChanged;

            public CameraRotationHelper(EnclosureLocation cameraEnclosureLocation)
            {
                _cameraEnclosureLocation = cameraEnclosureLocation;
                if (!IsEnclosureLocationExternal(_cameraEnclosureLocation))
                {
                    if (_orientationSensor != null)
                        _orientationSensor.OrientationChanged += SimpleOrientationSensor_OrientationChanged;
                }
                _displayInformation.OrientationChanged += DisplayInformation_OrientationChanged;
            }

            private void SimpleOrientationSensor_OrientationChanged(SimpleOrientationSensor sender, SimpleOrientationSensorOrientationChangedEventArgs args)
            {
                if (args.Orientation != SimpleOrientation.Faceup && args.Orientation != SimpleOrientation.Facedown)
                {
                    HandleOrientationChanged(false);
                }
            }

            private void DisplayInformation_OrientationChanged(DisplayInformation sender, object args)
            {
                HandleOrientationChanged(true);
            }

            private void HandleOrientationChanged(bool updatePreviewStreamRequired)
            {
                var handler = OrientationChanged;
                if (handler != null)
                {
                    handler(this, updatePreviewStreamRequired);
                }
            }

            public static bool IsEnclosureLocationExternal(EnclosureLocation enclosureLocation)
            {
                return (enclosureLocation == null || enclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Unknown);
            }

            private bool IsCameraMirrored()
            {
                // Front panel cameras are mirrored by default
                return (_cameraEnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
            }

            private SimpleOrientation GetCameraOrientationRelativeToNativeOrientation()
            {
                // Get the rotation angle of the camera enclosure
                var enclosureAngle = ConvertClockwiseDegreesToSimpleOrientation((int)_cameraEnclosureLocation.RotationAngleInDegreesClockwise);

                // Account for the fact that, on portrait-first devices, the built in camera sensor is read at a 90 degree offset to the native orientation
                if (_displayInformation.NativeOrientation == DisplayOrientations.Portrait && !IsEnclosureLocationExternal(_cameraEnclosureLocation))
                {
                    return AddOrientations(SimpleOrientation.Rotated90DegreesCounterclockwise, enclosureAngle);
                }
                else
                {
                    return AddOrientations(SimpleOrientation.NotRotated, enclosureAngle);
                }
            }

            // Gets the rotation to rotate ui elements
            public SimpleOrientation GetUIOrientation()
            {
                if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
                {
                    // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                    return SimpleOrientation.NotRotated;
                }

                // Return the difference between the orientation of the device and the orientation of the app display
                var deviceOrientation = _orientationSensor.GetCurrentOrientation();
                var displayOrientation = ConvertDisplayOrientationToSimpleOrientation(_displayInformation.CurrentOrientation);
                return SubOrientations(displayOrientation, deviceOrientation);
            }

            // Gets the rotation of the camera to rotate pictures/videos when saving to file
            public SimpleOrientation GetCameraCaptureOrientation()
            {
                if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
                {
                    // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                    return SimpleOrientation.NotRotated;
                }

                // Get the device orienation offset by the camera hardware offset
                var deviceOrientation = _orientationSensor.GetCurrentOrientation();
                var result = SubOrientations(deviceOrientation, GetCameraOrientationRelativeToNativeOrientation());

                // If the preview is being mirrored for a front-facing camera, then the rotation should be inverted
                if (IsCameraMirrored())
                {
                    result = MirrorOrientation(result);
                }
                return result;
            }

            // Gets the rotation of the camera to display the camera preview
            public SimpleOrientation GetCameraPreviewOrientation()
            {
                if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
                {
                    // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                    return SimpleOrientation.NotRotated;
                }

                // Get the app display rotation offset by the camera hardware offset
                var result = ConvertDisplayOrientationToSimpleOrientation(_displayInformation.CurrentOrientation);
                result = SubOrientations(result, GetCameraOrientationRelativeToNativeOrientation());

                // If the preview is being mirrored for a front-facing camera, then the rotation should be inverted
                if (IsCameraMirrored())
                {
                    result = MirrorOrientation(result);
                }
                return result;
            }

            public static PhotoOrientation ConvertSimpleOrientationToPhotoOrientation(SimpleOrientation orientation)
            {
                switch (orientation)
                {
                    case SimpleOrientation.Rotated90DegreesCounterclockwise:
                        return PhotoOrientation.Rotate90;
                    case SimpleOrientation.Rotated180DegreesCounterclockwise:
                        return PhotoOrientation.Rotate180;
                    case SimpleOrientation.Rotated270DegreesCounterclockwise:
                        return PhotoOrientation.Rotate270;
                    case SimpleOrientation.NotRotated:
                    default:
                        return PhotoOrientation.Normal;
                }
            }

            public static int ConvertSimpleOrientationToClockwiseDegrees(SimpleOrientation orientation)
            {
                switch (orientation)
                {
                    case SimpleOrientation.Rotated90DegreesCounterclockwise:
                        return 270;
                    case SimpleOrientation.Rotated180DegreesCounterclockwise:
                        return 180;
                    case SimpleOrientation.Rotated270DegreesCounterclockwise:
                        return 90;
                    case SimpleOrientation.NotRotated:
                    default:
                        return 0;
                }
            }

            private SimpleOrientation ConvertDisplayOrientationToSimpleOrientation(DisplayOrientations orientation)
            {
                SimpleOrientation result;
                switch (orientation)
                {
                    case DisplayOrientations.Landscape:
                        result = SimpleOrientation.NotRotated;
                        break;
                    case DisplayOrientations.PortraitFlipped:
                        result = SimpleOrientation.Rotated90DegreesCounterclockwise;
                        break;
                    case DisplayOrientations.LandscapeFlipped:
                        result = SimpleOrientation.Rotated180DegreesCounterclockwise;
                        break;
                    case DisplayOrientations.Portrait:
                    default:
                        result = SimpleOrientation.Rotated270DegreesCounterclockwise;
                        break;
                }

                // Above assumes landscape; offset is needed if native orientation is portrait
                if (_displayInformation.NativeOrientation == DisplayOrientations.Portrait)
                {
                    result = AddOrientations(result, SimpleOrientation.Rotated90DegreesCounterclockwise);
                }

                return result;
            }

            private static SimpleOrientation MirrorOrientation(SimpleOrientation orientation)
            {
                // This only affects the 90 and 270 degree cases, because rotating 0 and 180 degrees is the same clockwise and counter-clockwise
                switch (orientation)
                {
                    case SimpleOrientation.Rotated90DegreesCounterclockwise:
                        return SimpleOrientation.Rotated270DegreesCounterclockwise;
                    case SimpleOrientation.Rotated270DegreesCounterclockwise:
                        return SimpleOrientation.Rotated90DegreesCounterclockwise;
                }
                return orientation;
            }

            private static SimpleOrientation AddOrientations(SimpleOrientation a, SimpleOrientation b)
            {
                var aRot = ConvertSimpleOrientationToClockwiseDegrees(a);
                var bRot = ConvertSimpleOrientationToClockwiseDegrees(b);
                var result = (aRot + bRot) % 360;
                return ConvertClockwiseDegreesToSimpleOrientation(result);
            }

            private static SimpleOrientation SubOrientations(SimpleOrientation a, SimpleOrientation b)
            {
                var aRot = ConvertSimpleOrientationToClockwiseDegrees(a);
                var bRot = ConvertSimpleOrientationToClockwiseDegrees(b);
                //add 360 to ensure the modulus operator does not operate on a negative
                var result = (360 + (aRot - bRot)) % 360;
                return ConvertClockwiseDegreesToSimpleOrientation(result);
            }

            private static VideoRotation ConvertSimpleOrientationToVideoRotation(SimpleOrientation orientation)
            {
                switch (orientation)
                {
                    case SimpleOrientation.Rotated90DegreesCounterclockwise:
                        return VideoRotation.Clockwise270Degrees;
                    case SimpleOrientation.Rotated180DegreesCounterclockwise:
                        return VideoRotation.Clockwise180Degrees;
                    case SimpleOrientation.Rotated270DegreesCounterclockwise:
                        return VideoRotation.Clockwise90Degrees;
                    case SimpleOrientation.NotRotated:
                    default:
                        return VideoRotation.None;
                }
            }

            private static SimpleOrientation ConvertClockwiseDegreesToSimpleOrientation(int orientation)
            {
                switch (orientation)
                {
                    case 270:
                        return SimpleOrientation.Rotated90DegreesCounterclockwise;
                    case 180:
                        return SimpleOrientation.Rotated180DegreesCounterclockwise;
                    case 90:
                        return SimpleOrientation.Rotated270DegreesCounterclockwise;
                    case 0:
                    default:
                        return SimpleOrientation.NotRotated;
                }
            }
        }
    }
    public class ViewModel : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private static readonly PropertyChangedEventArgs ResultTextPropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(ResultText));

        private string resultText;

        public string ResultText
        {
            get { return this.resultText; }
            set
            {
                if (this.resultText == value) { return; }
                this.resultText = value;
                this.PropertyChanged?.Invoke(this, ResultTextPropertyChangedEventArgs);
            }
        }

        private static readonly PropertyChangedEventArgs ScanImagePropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(ScanImage));
        private SoftwareBitmapSource scanImage = new SoftwareBitmapSource();
        public SoftwareBitmapSource ScanImage
        {
            get { return this.scanImage; }
            set
            {
                if (this.scanImage == value) { return; }
                this.scanImage = value;
                this.PropertyChanged?.Invoke(this, ScanImagePropertyChangedEventArgs);
            }
        }


        private static readonly PropertyChangedEventArgs ScanHeightPropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(ScanHeight));
        private double scanheight;
        public double ScanHeight
        {
            get { return this.scanheight; }
            set
            {
                if (this.scanheight == value) { return; }
                this.scanheight = value;
                this.PropertyChanged?.Invoke(this, ScanHeightPropertyChangedEventArgs);
            }
        }

        private static readonly PropertyChangedEventArgs ThresholdPropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(Threshold));
        private int threshold = 0;
        public int Threshold
        {
            get { return this.threshold; }
            set
            {
                if (this.threshold == value) { return; }
                this.threshold = value;
                this.PropertyChanged?.Invoke(this, ThresholdPropertyChangedEventArgs);
            }
        }


        private static readonly PropertyChangedEventArgs MaxBritePropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(MaxBrite));
        private int maxBrite = 0;
        public int MaxBrite
        {
            get { return this.maxBrite; }
            set
            {
                if (this.maxBrite == value) { return; }
                this.maxBrite = value;
                this.PropertyChanged?.Invoke(this, MaxBritePropertyChangedEventArgs);
            }
        }

        private static readonly PropertyChangedEventArgs RangePropertyChangedEventArgs = new PropertyChangedEventArgs(nameof(Range));
        private double range = 0.5;
        public double Range
        {
            get { return this.range; }
            set
            {
                if (this.range == value) { return; }
                this.range = value;
                this.PropertyChanged?.Invoke(this, RangePropertyChangedEventArgs);
            }
        }


    }
    public static class TaslExtensions
    {
        public static void FireAndForget(this Task task)
        {
            task.ContinueWith(t => { Debug.WriteLine(t.Exception); }, TaskContinuationOptions.OnlyOnFaulted);
        }

        public static void FireAndForget(this IAsyncAction action)
        {
            action.AsTask().FireAndForget();
        }
    }

}