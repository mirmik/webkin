using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Controls.Primitives;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Media.Media3D;

using Transform = Kinematics.Transform;
using Transform3 = Kinematics.Transform3;
using Rotator3 = Kinematics.Rotator3;
using Actuator3 = Kinematics.Actuator3;
using Kinematic3 = Kinematics.KinematicTransform3OneScrew;
using Kinematics;

namespace KinematicModelConfigurator
{
    public enum SpatialTransformType
    {
        Transform,
        Rotator,
        Actuator
    }

    /// <summary>
    /// Логика взаимодействия для TreeElement.xaml
    /// </summary>
    public partial class TreeElement : UserControl
    {
        bool _isCollapsed = false;
        public bool IsCollapsed
        {
            get => _isCollapsed;
            set
            {
                _isCollapsed = value;
                OnPropertyChanged(nameof(IsCollapsed));
            }

            // get => ChildList.Visibility == Visibility.Collapsed;
            // set
            // {
            //     ChildList.Visibility = value ? Visibility.Collapsed : Visibility.Visible;
            //     OnPropertyChanged(nameof(IsCollapsed));
            // }
        }

        private void AxisControl_VectorChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("AxisControl_VectorChanged");
            UpdateAxis();
        }

        public void UpdateAxis()
        {
            if (transform is Kinematic3 k3d)
            {
                var axis = AxisControl.Vector;
                k3d.SetAxis(
                    new System.Numerics.Vector3((float)axis.X, (float)axis.Y, (float)axis.Z)
                );
                UpdateTransformParameters();
            }
        }

        string GenerateUniqueName()
        {
            return $"Element_{Guid.NewGuid().ToString().Substring(0, 8)}";
        }

        private void AddButton_Click(object sender, RoutedEventArgs e)
        {
            var newElement = new TreeElement(GenerateUniqueName());
            AddChild(newElement);
        }

        void RemakeTrihedronObjects()
        {
            if (_trihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_trihedronObject);
            }

            _trihedronObject = new TrihedronObject();
            KinematicModel3DControl.Instance.AddTrihedronObject(_trihedronObject);

            if (_assistedTrihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_assistedTrihedronObject);
            }

            if (transform is Kinematic3 k3d)
            {
                _assistedTrihedronObject = new TrihedronObject();
                KinematicModel3DControl.Instance.AddTrihedronObject(_assistedTrihedronObject);
            }

            TrihedronObjectPoseUpdate(is_top_level: true);
        }

        private void TreeElement_MouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
        {
            Console.WriteLine($"Mouse entered on TreeElement '{ElementName}'");
            _trihedronObject = new TrihedronObject();
            KinematicModel3DControl.Instance.AddTrihedronObject(_trihedronObject);

            if (transform is Kinematic3 k3d)
            {
                _assistedTrihedronObject = new TrihedronObject();
                KinematicModel3DControl.Instance.AddTrihedronObject(_assistedTrihedronObject);
            }

            TrihedronObjectPoseUpdate(is_top_level: true);
            _hovered = true;
        }

        void TrihedronObjectPoseUpdate(bool is_top_level = false)
        {
            if (_trihedronObject == null)
                return;

            Pose3 pose = transform.GlobalPose();
            _trihedronObject.SetPose(pose);

            if (_assistedTrihedronObject != null && transform is Kinematic3 k3d)
            {
                Console.WriteLine("Updating assisted trihedron pose");
                var k3d_child = k3d.Children[0];
                Pose3 assistedPose = k3d_child.GlobalPose();
                _assistedTrihedronObject.SetPose(assistedPose);
            }

            if (!is_top_level)
            {
                _trihedronObject.SetHalfColor();
            }
            else
            {
                _trihedronObject.SetFullColor();
            }

            Parent?.TrihedronObjectPoseUpdate();
        }

        private void TreeElement_MouseLeave(object sender, System.Windows.Input.MouseEventArgs e)
        {
            Console.WriteLine($"Mouse left on TreeElement '{ElementName}'");
            if (_trihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_trihedronObject);
                _trihedronObject = null;
            }

            if (_assistedTrihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_assistedTrihedronObject);
                _assistedTrihedronObject = null;
            }

            Parent?.TrihedronObjectPoseUpdate(is_top_level: true);
            _hovered = false;
        }

        private void PositionControl_VectorChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("PositionControl_VectorChanged");
            ModelUpdate();
        }

        private void RotationControl_VectorChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("RotationControl_VectorChanged");
            ModelUpdate();
        }

        private void CubeSizeControl_VectorChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("CubeSizeControl_VectorChanged");
            ModelUpdate();
        }

        private void SphereSizeControl_OnRadiusChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("SphereSizeControl_OnRadiusChanged");
            ModelUpdate();
        }

        private void CylinderSizeControl_OnSizeChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("CylinderSizeControl_OnSizeChanged");
            ModelUpdate();
        }

        private void ConeSizeControl_OnSizeChanged(object? sender, EventArgs e)
        {
            Console.WriteLine("ConeSizeControl_OnSizeChanged");
            ModelUpdate();
        }

        public TreeElement? FindElementByName(string name)
        {
            if (ElementName == name)
                return this;

            foreach (var child in Children)
            {
                var result = child.FindElementByName(name);
                if (result != null)
                    return result;
            }

            return null;
        }

        public void RecurseModelUpdate()
        {
            ModelUpdate();
            foreach (var child in Children)
            {
                child.RecurseModelUpdate();
            }
        }

        public ObservableCollection<TreeElement> Children { get; set; } = new();
        InteractiveObject? _interactiveObject;
        TrihedronObject? _trihedronObject;
        TrihedronObject? _assistedTrihedronObject;

        float _minForRotator = -180f; // 180 degrees
        float _maxForRotator = 180f;
        float _minForActuator = -500f; // 5 meters
        float _maxForActuator = 500f;
        bool _hovered = false;
        private const string TreeElementDragDataFormat = "TreeElementDragData";
        private Point _dragStartPoint;
        private bool _isMouseDown;
        private InsertionAdorner? _insertionAdorner;

        private string _elementName = "Element";
        public string ElementName
        {
            get => _elementName;
            set
            {
                if (_elementName == value)
                    return;
                _elementName = value;
                if (transform != null)
                {
                    transform.Name = _elementName;
                }
                OnPropertyChanged(nameof(ElementName));
            }
        }
        public Transform transform = new Transform3();

        private string _objFilePath = "";
        public string ObjFilePath
        {
            get => _objFilePath;
            set
            {
                _objFilePath = value;
                OnPropertyChanged(nameof(ObjFilePath));
                ModelUpdate();
            }
        }

        private void ModelTypeComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ModelUpdate();
        }

        private void SpartialType_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ModelUpdate();

            if (_hovered)
                RemakeTrihedronObjects();

            if (NonInited())
                return;

            // Update MinMaxSlider limits based on selected spatial type
            var spartialType = (SpatialTransformType)((ComboBox)SpartialTypeCBI).SelectedItem;
            if (spartialType == SpatialTransformType.Rotator)
            {
                SpartialSlider.UpdateLimits(_minForRotator, _maxForRotator);
                SpartialSlider.SetValue(0); // or some default value
            }
            else if (spartialType == SpatialTransformType.Actuator)
            {
                SpartialSlider.UpdateLimits(_minForActuator, _maxForActuator);
                SpartialSlider.SetValue(0); // or some default value
            }
            else
            {
                SpartialSlider.UpdateLimits(0, 0); // or some default values
            }
        }

        bool NonInited()
        {
            return CubeSizeControl == null || PositionControl == null || RotationControl == null;
        }

        List<Transform> UnbindChildren()
        {
            Transform me = transform;

            if (transform is Kinematic3 k3d)
            {
                me = k3d.Children[0];
            }

            List<Transform> childs = me.Children.ToList();
            for (int i = childs.Count - 1; i >= 0; i--)
            {
                childs[i].Unparent();
            }

            return childs;
        }

        void UpdateTransform()
        {
            List<Transform> childs = UnbindChildren();
            var parent = transform.Parent;
            if (parent != null)
                transform.Unparent();

            // currentSpatialType from UI
            var currentSpatialType_cbi = (SpatialTransformType)
                ((ComboBox)SpartialTypeCBI).SelectedItem;
            //string? currentSpatialType_str = currentSpatialType_cbi?.Content as string;

            //if (currentSpatialType_str == null)
            //    throw new Exception("Failed to get current spatial type from UI");

            SpatialTransformType currentSpatialType = currentSpatialType_cbi switch
            {
                SpatialTransformType.Rotator => SpatialTransformType.Rotator,
                SpatialTransformType.Actuator => SpatialTransformType.Actuator,
                _ => SpatialTransformType.Transform,
            };

            var axis = AxisControl.Vector;

            Transform newTransform = currentSpatialType switch
            {
                SpatialTransformType.Transform => new Transform3(),
                SpatialTransformType.Rotator
                    => new Rotator3(
                        new System.Numerics.Vector3((float)axis.X, (float)axis.Y, (float)axis.Z)
                    ),
                SpatialTransformType.Actuator
                    => new Actuator3(
                        new System.Numerics.Vector3((float)axis.X, (float)axis.Y, (float)axis.Z)
                    ),
                _ => new Transform3(),
            };

            if (parent != null)
            {
                parent.Link(newTransform);
            }

            transform = newTransform;
            transform.Name = ElementName;

            foreach (var child in childs)
            {
                transform.Link(child);
            }

            UpdateTransformParameters();
        }

        public void UpdateTransformParameters()
        {
            Console.WriteLine("UpdateTransformParameters()");

            var lin = PositionControl.Vector;
            var eul = RotationControl.Vector;

            Pose3 result =
                Pose3.Translation((float)lin.X, (float)lin.Y, (float)lin.Z)
                * Pose3.RotateX((float)(eul.X * Math.PI / 180.0))
                * Pose3.RotateY((float)(eul.Y * Math.PI / 180.0))
                * Pose3.RotateZ((float)(eul.Z * Math.PI / 180.0));

            transform.Relocate(result);

            if (transform is Kinematic3 k3d)
            {
                Console.WriteLine("Setting Kinematic3 coord");
                Console.WriteLine($"Slider value: {SpartialSlider.Value}");
                // slider value
                var sliderValue = SpartialSlider.Value;

                SpatialTransformType spartialType = (SpatialTransformType)
                    ((ComboBox)SpartialTypeCBI).SelectedItem;
                float mul = spartialType switch
                {
                    SpatialTransformType.Rotator => MathF.PI / 180f,
                    SpatialTransformType.Actuator => 0.01f,
                    _ => 1.0f,
                };

                k3d.SetCoord((float)sliderValue * mul);

                TrihedronObjectPoseUpdate(is_top_level: true);
            }

            RelocateInteractivesRecursive();
        }

        public void RelocateInteractivesRecursive()
        {
            Pose3 pose = transform.GlobalPose();

            if (_interactiveObject != null)
            {
                _interactiveObject.SetPose(pose);
            }

            foreach (var child in Children)
            {
                child.RelocateInteractivesRecursive();
            }
        }

        public string ModelType
        {
            get
            {
                var modelTypeComboBox = this.FindName("ModelTypeComboBox") as ComboBox;
                string ModelType =
                    ((ComboBoxItem?)(modelTypeComboBox?.SelectedItem))?.Content as string ?? "Cube";
                return ModelType;
            }
        }

        public void ModelUpdate()
        {
            if (NonInited())
                return;

            UpdateTransform();

            // Get ModelTypeComboBox value
            var modelTypeComboBox = this.FindName("ModelTypeComboBox") as ComboBox;
            string ModelType =
                ((ComboBoxItem?)(modelTypeComboBox?.SelectedItem))?.Content as string ?? "Cube";

            if (_interactiveObject != null)
            {
                // Remove old object from KinematicModel3DControl
                KinematicModel3DControl.Instance.RemoveInteractiveObject(_interactiveObject);
            }
            if (ModelType == "Cube")
            {
                Vector3D size = CubeSizeControl.Vector;
                _interactiveObject = InteractiveObject.CreateCube(size.X, size.Y, size.Z);
                KinematicModel3DControl.Instance.AddInteractiveObject(_interactiveObject);
                Pose3 pose = transform.GlobalPose();
                _interactiveObject.SetPose(pose);
            }
            else if (ModelType == "Sphere")
            {
                double radius = SphereSizeControl.Radius;
                _interactiveObject = InteractiveObject.CreateSphere(radius);
                KinematicModel3DControl.Instance.AddInteractiveObject(_interactiveObject);
                Pose3 pose = transform.GlobalPose();
                _interactiveObject.SetPose(pose);
            }
            else if (ModelType == "Cylinder")
            {
                double radius = CylinderSizeControl.Radius;
                double height = CylinderSizeControl.Height;
                _interactiveObject = InteractiveObject.CreateCylinder(radius, height);
                KinematicModel3DControl.Instance.AddInteractiveObject(_interactiveObject);
                Pose3 pose = transform.GlobalPose();
                _interactiveObject.SetPose(pose);
            }
            else if (ModelType == "Cone")
            {
                double radius1 = ConeSizeControl.Radius1;
                double radius2 = ConeSizeControl.Radius2;
                double height = ConeSizeControl.Height;
                _interactiveObject = InteractiveObject.CreateCone(radius1, radius2, height);
                KinematicModel3DControl.Instance.AddInteractiveObject(_interactiveObject);
                Pose3 pose = transform.GlobalPose();
                _interactiveObject.SetPose(pose);
            }
            else if (ModelType == "File" && !string.IsNullOrEmpty(ObjFilePath))
            {
                var fullPath = System.IO.Path.Combine(
                    ArchiveManager.GetTempDirectory(),
                    ObjFilePath
                );
                _interactiveObject = InteractiveObject.CreateFromObjFile(fullPath);
                KinematicModel3DControl.Instance.AddInteractiveObject(_interactiveObject);
                Pose3 pose = transform.GlobalPose();
                _interactiveObject.SetPose(pose);
            }

            CheckTransformTreeValidity();
            TrihedronObjectPoseUpdate(is_top_level: true);
        }

        public void CheckValidityRecursivly()
        {
            CheckTransformTreeValidity();

            foreach (var child in Children)
            {
                child.CheckValidityRecursivly();
            }
        }

        public List<string> GetChildNamesFromTransform()
        {
            Transform me = transform;

            if (transform is Kinematic3 k3d)
            {
                me = k3d.Children[0];
            }

            List<string> childNames = me.Children.Select(c => c.Name).ToList();
            return childNames;
        }

        List<string> GetChildNamesFromUI()
        {
            List<string> childNames = Children.Select(c => c.ElementName).ToList();
            return childNames;
        }

        void CheckTransformTreeValidity()
        {
            // Проверяем корректность дерева трансформов
            string myname = ElementName;
            string transform_name = transform.Name;
            if (myname != transform_name)
            {
                throw new Exception(
                    $"Transform name '{transform_name}' does not match TreeElement name '{myname}'"
                );
            }

            List<string> transformChildNames = GetChildNamesFromTransform();
            List<string> uiChildNames = GetChildNamesFromUI();

            if (transformChildNames.Count != uiChildNames.Count)
            {
                throw new Exception(
                    $"Transform '{myname}' has {transformChildNames.Count} children, but UI has {uiChildNames.Count} children"
                );
            }

            // compare as sets
            var transformChildNamesSet = new HashSet<string>(transformChildNames);
            var uiChildNamesSet = new HashSet<string>(uiChildNames);
            if (!transformChildNamesSet.SetEquals(uiChildNamesSet))
            {
                throw new Exception(
                    $"Transform '{myname}' children names do not match UI children names"
                );
            }
        }

        public event System.ComponentModel.PropertyChangedEventHandler? PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(
                this,
                new System.ComponentModel.PropertyChangedEventArgs(propertyName)
            );
        }

        public TreeElement(string name)
        {
            DataContext = this;
            InitializeComponent();
            Parent = null;
            ElementName = name;
            //BrowseObjFileCommand = new RelayCommand(_ => BrowseObjFile());
            // ensure TextBox shows the initial value
            ElementNameTextBox.Text = ElementName;

            PositionControl.VectorChanged += PositionControl_VectorChanged;
            RotationControl.VectorChanged += RotationControl_VectorChanged;
            CubeSizeControl.VectorChanged += CubeSizeControl_VectorChanged;
            SphereSizeControl.RadiusChanged += SphereSizeControl_OnRadiusChanged;
            CylinderSizeControl.RadiusChanged += CylinderSizeControl_OnSizeChanged;
            CylinderSizeControl.HeightChanged += CylinderSizeControl_OnSizeChanged;
            ConeSizeControl.Radius1Changed += ConeSizeControl_OnSizeChanged;
            ConeSizeControl.Radius2Changed += ConeSizeControl_OnSizeChanged;
            ConeSizeControl.HeightChanged += ConeSizeControl_OnSizeChanged;

            ModelUpdate();
            transform.Name = ElementName;
            Loaded += (s, e) =>
            {
                ModelUpdate();
                transform.Name = ElementName;
                var minMaxSlider = SpartialSlider;
                if (minMaxSlider != null)
                    minMaxSlider.ValueChanged += MinMaxSlider_ValueChanged;
            };
        }

        public void LoadedRecursively()
        {
            ModelUpdate();
            foreach (var child in Children)
            {
                child.LoadedRecursively();
            }
        }

        private void MinMaxSlider_ValueChanged(
            object sender,
            RoutedPropertyChangedEventArgs<double> e
        )
        {
            var kinunit = transform as Kinematic3;
            if (kinunit != null)
            {
                Console.WriteLine("MinMaxSlider_ValueChanged");
                // For example, set the actuator position based on the slider value
                double sliderValue = e.NewValue;
                kinunit.SetCoord((float)sliderValue);
                ModelUpdate();
            }
        }

        public object ToTrent()
        {
            return ToTrentWithChildren();
        }

        public new TreeElement? Parent { get; set; }

        public void AddChild(TreeElement child)
        {
            InsertChild(Children.Count, child);
        }

        public void InsertChild(int index, TreeElement child)
        {
            child.Parent = this;
            index = Math.Max(0, Math.Min(index, Children.Count));
            Children.Insert(index, child);

            var childTransform = child.transform;
            transform.Link(childTransform);

            CheckValidityRecursivly();
        }

        private void ChildList_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (!ShouldInitiateDrag(e.OriginalSource as DependencyObject))
            {
                _isMouseDown = false;
                return;
            }

            _dragStartPoint = e.GetPosition(null);
            _isMouseDown = true;
        }

        private void ChildList_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            _isMouseDown = false;
        }

        private void ChildList_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (!_isMouseDown || e.LeftButton != MouseButtonState.Pressed)
                return;

            var currentPosition = e.GetPosition(null);
            if (
                Math.Abs(currentPosition.X - _dragStartPoint.X)
                    < SystemParameters.MinimumHorizontalDragDistance
                && Math.Abs(currentPosition.Y - _dragStartPoint.Y)
                    < SystemParameters.MinimumVerticalDragDistance
            )
            {
                return;
            }

            _isMouseDown = false;

            if (sender is ListBox listBox)
            {
                StartDrag(listBox, e);
            }
        }

        private void StartDrag(ListBox listBox, MouseEventArgs e)
        {
            var listBoxItem = FindVisualParent<ListBoxItem>(e.OriginalSource as DependencyObject);
            if (listBoxItem == null)
                return;

            var draggedElement =
                listBox.ItemContainerGenerator.ItemFromContainer(listBoxItem) as TreeElement;
            if (draggedElement == null)
                return;

            var dragData = new DataObject(TreeElementDragDataFormat, draggedElement);
            DragDrop.DoDragDrop(listBoxItem, dragData, DragDropEffects.Move);
        }

        private void ChildList_DragOver(object sender, DragEventArgs e)
        {
            if (sender is not ListBox listBox)
            {
                e.Effects = DragDropEffects.None;
                e.Handled = true;
                return;
            }

            if (!TryGetDraggedElement(e, out var draggedElement) || !CanAcceptDrop(draggedElement))
            {
                e.Effects = DragDropEffects.None;
                RemoveInsertionAdorner();
            }
            else
            {
                e.Effects = DragDropEffects.Move;
                var targetItem = GetItemFromEvent(listBox, e);
                UpdateInsertionAdorner(listBox, targetItem, e);
            }

            e.Handled = true;
        }

        private void ChildList_Drop(object sender, DragEventArgs e)
        {
            if (
                !TryGetDraggedElement(e, out var draggedElement)
                || !CanAcceptDrop(draggedElement)
                || sender is not ListBox listBox
            )
            {
                e.Handled = true;
                return;
            }

            var targetItem = GetItemFromEvent(listBox, e);
            int insertIndex = CalculateInsertIndex(listBox, targetItem, e);

            MoveElementToTarget(draggedElement, this, insertIndex);

            RemoveInsertionAdorner();

            e.Handled = true;
        }

        private void ChildList_DragLeave(object sender, DragEventArgs e)
        {
            RemoveInsertionAdorner();
        }

        private static bool TryGetDraggedElement(DragEventArgs e, out TreeElement? draggedElement)
        {
            draggedElement = null;
            if (!e.Data.GetDataPresent(TreeElementDragDataFormat))
                return false;

            draggedElement = e.Data.GetData(TreeElementDragDataFormat) as TreeElement;
            return draggedElement != null;
        }

        private bool CanAcceptDrop(TreeElement draggedElement)
        {
            if (draggedElement == this)
                return false;

            return !IsAncestor(draggedElement, this);
        }

        private static bool IsAncestor(TreeElement potentialAncestor, TreeElement node)
        {
            var current = node.Parent;
            while (current != null)
            {
                if (current == potentialAncestor)
                    return true;
                current = current.Parent;
            }

            return false;
        }

        private static TreeElement? GetItemFromEvent(ListBox listBox, DragEventArgs e)
        {
            var listBoxItem = FindVisualParent<ListBoxItem>(e.OriginalSource as DependencyObject);
            if (listBoxItem == null)
                return null;

            return listBox.ItemContainerGenerator.ItemFromContainer(listBoxItem) as TreeElement;
        }

        private static int CalculateInsertIndex(
            ListBox listBox,
            TreeElement? targetItem,
            DragEventArgs e
        )
        {
            int insertIndex = listBox.Items.Count;
            if (targetItem != null)
            {
                insertIndex = listBox.Items.IndexOf(targetItem);
                var container =
                    listBox.ItemContainerGenerator.ContainerFromItem(targetItem) as ListBoxItem;
                if (container != null)
                {
                    var position = e.GetPosition(container);
                    if (position.Y > container.ActualHeight / 2)
                    {
                        insertIndex++;
                    }
                }
            }

            return Math.Max(0, Math.Min(insertIndex, listBox.Items.Count));
        }

        private bool ShouldInitiateDrag(DependencyObject? source)
        {
            if (source == null)
                return false;

            if (IsWithinInteractiveElement(source))
                return false;

            return FindVisualParent<ListBoxItem>(source) != null;
        }

        private void UpdateInsertionAdorner(
            ListBox listBox,
            TreeElement? targetItem,
            DragEventArgs e
        )
        {
            UIElement? adornedElement = null;
            bool insertAfter = false;

            if (targetItem != null)
            {
                adornedElement =
                    listBox.ItemContainerGenerator.ContainerFromItem(targetItem) as UIElement;
                if (adornedElement != null)
                {
                    var position = e.GetPosition(adornedElement);
                    insertAfter = position.Y > adornedElement.RenderSize.Height / 2;
                }
            }
            else if (listBox.Items.Count > 0)
            {
                var lastItem = listBox.Items[listBox.Items.Count - 1];
                adornedElement =
                    listBox.ItemContainerGenerator.ContainerFromItem(lastItem) as UIElement;
                insertAfter = true;
            }
            else
            {
                adornedElement = listBox;
                insertAfter = false;
            }

            if (adornedElement != null)
            {
                ShowInsertionAdorner(adornedElement, insertAfter);
            }
            else
            {
                RemoveInsertionAdorner();
            }
        }

        private void ShowInsertionAdorner(UIElement adornedElement, bool insertAfter)
        {
            if (
                _insertionAdorner != null
                && _insertionAdorner.AdornedElement == adornedElement
                && _insertionAdorner.IsAfter == insertAfter
            )
            {
                return;
            }

            RemoveInsertionAdorner();
            var adornerLayer = AdornerLayer.GetAdornerLayer(adornedElement);
            if (adornerLayer == null)
                return;

            _insertionAdorner = new InsertionAdorner(adornedElement, insertAfter);
            adornerLayer.Add(_insertionAdorner);
        }

        private void RemoveInsertionAdorner()
        {
            if (_insertionAdorner == null)
                return;

            var adornerLayer = AdornerLayer.GetAdornerLayer(_insertionAdorner.AdornedElement);
            if (adornerLayer != null)
            {
                adornerLayer.Remove(_insertionAdorner);
            }

            _insertionAdorner = null;
        }

        private static bool IsWithinInteractiveElement(DependencyObject source)
        {
            while (source != null)
            {
                if (
                    source is TextBoxBase
                    || source is PasswordBox
                    || source is ComboBox
                    || source is Slider
                    || source is ButtonBase
                    || source is ScrollBar
                )
                {
                    return true;
                }

                source = VisualTreeHelper.GetParent(source);
            }

            return false;
        }

        private static void MoveElementToTarget(
            TreeElement draggedElement,
            TreeElement targetParent,
            int insertIndex
        )
        {
            var sourceParent = draggedElement.Parent;
            if (sourceParent == null)
                return;

            if (sourceParent == targetParent)
            {
                int currentIndex = sourceParent.Children.IndexOf(draggedElement);
                if (currentIndex < 0)
                    return;

                insertIndex = Math.Max(0, Math.Min(insertIndex, sourceParent.Children.Count));
                if (currentIndex < insertIndex)
                {
                    insertIndex = Math.Max(0, insertIndex - 1);
                }

                insertIndex = Math.Min(insertIndex, sourceParent.Children.Count - 1);
                if (currentIndex != insertIndex)
                {
                    sourceParent.Children.Move(currentIndex, insertIndex);
                }
            }
            else
            {
                draggedElement.Unparent();
                targetParent.InsertChild(insertIndex, draggedElement);
                if (sourceParent != null)
                {
                    sourceParent.CheckValidityRecursivly();
                }
            }

            draggedElement.RecurseModelUpdate();
        }

        private static T? FindVisualParent<T>(DependencyObject? source) where T : DependencyObject
        {
            while (source != null)
            {
                if (source is T parent)
                    return parent;
                source = VisualTreeHelper.GetParent(source);
            }

            return null;
        }

        private sealed class InsertionAdorner : Adorner
        {
            private readonly bool _isAfter;
            private readonly Pen _pen;

            public bool IsAfter => _isAfter;

            public InsertionAdorner(UIElement adornedElement, bool isAfter) : base(adornedElement)
            {
                _isAfter = isAfter;
                var brush = new SolidColorBrush(Color.FromRgb(0x33, 0x99, 0xFF));
                brush.Freeze();
                _pen = new Pen(brush, 2);
                _pen.Freeze();
                IsHitTestVisible = false;
            }

            protected override void OnRender(DrawingContext drawingContext)
            {
                base.OnRender(drawingContext);
                double y = _isAfter ? AdornedElement.RenderSize.Height : 0;
                var start = new Point(0, y);
                var end = new Point(AdornedElement.RenderSize.Width, y);
                drawingContext.DrawLine(_pen, start, end);
            }
        }

        void Unparent()
        {
            if (Parent != null)
            {
                Parent.Children.Remove(this);
                Parent = null;
            }

            transform.Unparent();
        }

        public static void ToEulerAngles(
            System.Numerics.Quaternion q,
            out float roll,
            out float pitch,
            out float yaw
        )
        {
            // roll (x-axis rotation)
            float sinr_cosp = 2 * (q.W * q.X + q.Y * q.Z);
            float cosr_cosp = 1 - 2 * (q.X * q.X + q.Y * q.Y);
            roll = MathF.Atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            float sinp = 2 * (q.W * q.Y - q.Z * q.X);
            if (MathF.Abs(sinp) >= 1)
                pitch = MathF.CopySign(MathF.PI / 2, sinp); // use 90 degrees if out of range
            else
                pitch = MathF.Asin(sinp);

            // yaw (z-axis rotation)
            float siny_cosp = 2 * (q.W * q.Z + q.X * q.Y);
            float cosy_cosp = 1 - 2 * (q.Y * q.Y + q.Z * q.Z);
            yaw = MathF.Atan2(siny_cosp, cosy_cosp);
        }

        public void SetLocalPose(Pose3 pose)
        {
            ((XYZControl)PositionControl).Vector = new Vector3D(pose.Lin.X, pose.Lin.Y, pose.Lin.Z);

            // Extract Euler angles from rotation matrix
            var quat = pose.Ang;

            float roll,
                pitch,
                yaw;

            ToEulerAngles(quat, out roll, out pitch, out yaw);

            ((XYZControl)RotationControl).Vector = new Vector3D(
                roll * 180.0 / Math.PI,
                pitch * 180.0 / Math.PI,
                yaw * 180.0 / Math.PI
            );
        }

        // public static TreeElement FromTransform(Transform tr)
        // {
        //     var name = tr.Name;
        //     TreeElement tre = new TreeElement(name);
        //     Pose3 local_pose = tr.LocalPose;
        //     tre.SetLocalPose(local_pose);

        //     foreach (var ch in tr.Children)
        //     {
        //         tre.AddChild(FromTransform(ch));
        //     }

        //     return tre;
        // }

        public void SetTransformType(SpatialTransformType type)
        {
            var spartialTypeComboBox = this.FindName("SpartialTypeCBI") as ComboBox;
            if (spartialTypeComboBox == null)
                throw new Exception("Failed to find SpartialTypeCBI ComboBox");
            spartialTypeComboBox.SelectedItem = type;
        }

        public static TreeElement FromTrent(Dictionary<string, object> trentDct)
        {
            string name = trentDct.ContainsKey("name")
                ? trentDct["name"] as string ?? "Element"
                : "Element";
            TreeElement tre = new TreeElement(name);

            var type = trentDct.ContainsKey("type")
                ? trentDct["type"] as string ?? "transform"
                : "transform";

            var type_enum = type.ToLower() switch
            {
                "rotator" => SpatialTransformType.Rotator,
                "actuator" => SpatialTransformType.Actuator,
                _ => SpatialTransformType.Transform,
            };

            tre.SetTransformType(type_enum);

            if (trentDct.ContainsKey("pose"))
            {
                var poseDct = trentDct["pose"] as Dictionary<string, object>;
                if (poseDct != null)
                {
                    var positionList = poseDct["position"] as List<object>;
                    var orientationList = poseDct["orientation"] as List<object>;

                    Pose3 pose = new Pose3(
                        lin: new System.Numerics.Vector3(),
                        ang: new System.Numerics.Quaternion()
                    );

                    if (positionList != null && positionList.Count == 3)
                    {
                        pose.Lin = new System.Numerics.Vector3(
                            Convert.ToSingle(positionList[0]),
                            Convert.ToSingle(positionList[1]),
                            Convert.ToSingle(positionList[2])
                        );
                    }

                    if (orientationList != null && orientationList.Count == 4)
                    {
                        pose.Ang = new System.Numerics.Quaternion(
                            Convert.ToSingle(orientationList[0]),
                            Convert.ToSingle(orientationList[1]),
                            Convert.ToSingle(orientationList[2]),
                            Convert.ToSingle(orientationList[3])
                        );
                    }

                    tre.SetLocalPose(pose);
                }
            }

            if (trentDct.ContainsKey("axis"))
            {
                var axisList = trentDct["axis"] as List<object>;
                if (axisList != null && axisList.Count == 3)
                {
                    tre.AxisControl.Vector = new Vector3D(
                        Convert.ToDouble(axisList[0]),
                        Convert.ToDouble(axisList[1]),
                        Convert.ToDouble(axisList[2])
                    );
                }
            }

            if (trentDct.ContainsKey("children"))
            {
                var childrenList = trentDct["children"] as List<object>;
                if (childrenList != null)
                {
                    foreach (var childObj in childrenList)
                    {
                        var childDct = childObj as Dictionary<string, object>;
                        if (childDct != null)
                        {
                            tre.AddChild(FromTrent(childDct));
                        }
                    }
                }
            }

            if (trentDct.ContainsKey("model"))
            {
                var modelDct = trentDct["model"] as Dictionary<string, object>;
                if (modelDct != null)
                {
                    if (modelDct.ContainsKey("type"))
                    {
                        string modelType = modelDct["type"] as string ?? "Cube";
                        var modelTypeComboBox = tre.FindName("ModelTypeComboBox") as ComboBox;
                        if (modelTypeComboBox != null)
                        {
                            foreach (ComboBoxItem item in modelTypeComboBox.Items)
                            {
                                var cstr = item.Content as string;
                                var lower_cstr = cstr?.ToLower() ?? "";
                                if (lower_cstr == modelType.ToLower())
                                {
                                    modelTypeComboBox.SelectedItem = item;
                                    break;
                                }
                            }
                        }
                    }

                    var model_type = modelDct.ContainsKey("type")
                        ? modelDct["type"] as string ?? "Cube"
                        : "Cube";
                    model_type = model_type.Trim().ToLower() switch
                    {
                        "file" => "File",
                        "cube" => "Cube",
                        "sphere" => "Sphere",
                        "cylinder" => "Cylinder",
                        "cone" => "Cone",
                        _ => "Cube",
                    };

                    if (model_type == "File" && modelDct.ContainsKey("path"))
                    {
                        string path = modelDct["path"] as string ?? "";
                        tre.ObjFilePath = path;
                    }
                    else if (model_type == "Cube" && modelDct.ContainsKey("size"))
                    {
                        var sizeList = modelDct["size"] as List<object>;
                        if (sizeList != null && sizeList.Count == 3)
                        {
                            tre.CubeSizeControl.Vector = new Vector3D(
                                Convert.ToDouble(sizeList[0]),
                                Convert.ToDouble(sizeList[1]),
                                Convert.ToDouble(sizeList[2])
                            );
                        }
                    }
                    else if (model_type == "Sphere" && modelDct.ContainsKey("radius"))
                    {
                        double radius = Convert.ToDouble(modelDct["radius"]);
                        tre.SphereSizeControl.Radius = radius;
                    }
                    else if (
                        model_type == "Cylinder"
                        && modelDct.ContainsKey("radius")
                        && modelDct.ContainsKey("height")
                    )
                    {
                        double radius = Convert.ToDouble(modelDct["radius"]);
                        double height = Convert.ToDouble(modelDct["height"]);
                        tre.CylinderSizeControl.Radius = radius;
                        tre.CylinderSizeControl.Height = height;
                    }
                    else if (
                        model_type == "Cone"
                        && modelDct.ContainsKey("radius1")
                        && modelDct.ContainsKey("radius2")
                        && modelDct.ContainsKey("height")
                    )
                    {
                        double radius1 = Convert.ToDouble(modelDct["radius1"]);
                        double radius2 = Convert.ToDouble(modelDct["radius2"]);
                        double height = Convert.ToDouble(modelDct["height"]);
                        tre.ConeSizeControl.Radius1 = radius1;
                        tre.ConeSizeControl.Radius2 = radius2;
                        tre.ConeSizeControl.Height = height;
                    }
                }
            }

            return tre;
        }

        public void CleanupRecursivly()
        {
            if (_interactiveObject != null)
            {
                KinematicModel3DControl.Instance.RemoveInteractiveObject(_interactiveObject);
                _interactiveObject = null;
            }

            if (_trihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_trihedronObject);
                _trihedronObject = null;
            }

            if (_assistedTrihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_assistedTrihedronObject);
                _assistedTrihedronObject = null;
            }

            transform.Unparent();
            transform = null!;

            foreach (var child in Children)
            {
                child.CleanupRecursivly();
            }
        }

        void CleanUpGraphics()
        {
            if (_interactiveObject != null)
            {
                KinematicModel3DControl.Instance.RemoveInteractiveObject(_interactiveObject);
                _interactiveObject = null;
            }

            if (_trihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_trihedronObject);
                _trihedronObject = null;
            }

            if (_assistedTrihedronObject != null)
            {
                KinematicModel3DControl.Instance.RemoveTrihedronObject(_assistedTrihedronObject);
                _assistedTrihedronObject = null;
            }
        }

        private void DeleteButton_Click(object sender, RoutedEventArgs e)
        {
            CleanUpGraphics();

            List<TreeElement> childElements = Children.ToList();
            foreach (var child in childElements)
            {
                child.Unparent();
            }

            if (Parent != null)
            {
                transform.Unparent();
                Parent.Children.Remove(this);
            }
            else
            {
                MessageBox.Show(
                    "Нельзя удалить корневой элемент.",
                    "Ошибка",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error
                );
            }

            foreach (var child in childElements)
            {
                Parent?.AddChild(child);
            }
        }

        private void MoveUpButton_Click(object sender, RoutedEventArgs e)
        {
            if (Parent == null)
                return;
            var index = Parent.Children.IndexOf(this);
            if (index > 0)
            {
                Parent.Children.Move(index, index - 1);
            }
        }

        private void MoveDownButton_Click(object sender, RoutedEventArgs e)
        {
            if (Parent == null)
                return;
            var index = Parent.Children.IndexOf(this);
            if (index < Parent.Children.Count - 1)
            {
                Parent.Children.Move(index, index + 1);
            }
        }

        // private void BrowseObjFile()
        // {
        //     var dlg = new Microsoft.Win32.OpenFileDialog();
        //     dlg.Filter = "OBJ files (*.obj)|*.obj|All files (*.*)|*.*";
        //     if (dlg.ShowDialog() == true)
        //     {
        //         ObjFilePath = dlg.FileName;
        //     }
        // }

        // private void BrowseObjFile_Click(object sender, RoutedEventArgs e)
        // {
        //     BrowseObjFile();
        // }

        private void ObjFileComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            var combo = sender as ComboBox;
            FileEntry? fileEntry = combo?.SelectedItem as FileEntry;
            if (fileEntry?.Path is string path)
            {
                ObjFilePath = path;
            }
        }

        void CollectModelPathes_Impl(List<string> pathes)
        {
            if (ModelType == "File" && !string.IsNullOrEmpty(ObjFilePath))
            {
                if (!pathes.Contains(ObjFilePath))
                {
                    pathes.Add(ObjFilePath);
                }
            }

            foreach (var child in Children)
            {
                child.CollectModelPathes_Impl(pathes);
            }
        }

        public List<string> CollectModelPathes()
        {
            List<string> pathes = new List<string>();
            CollectModelPathes_Impl(pathes);
            return pathes;
        }

        public Dictionary<string, object> ToTrentWithChildren(bool topWithoutPose = false)
        {
            var local_pose = transform.LocalPose;

            string type = transform switch
            {
                Rotator3 => "rotator",
                Actuator3 => "actuator",
                _ => "transform",
            };

            List<object> childrenList = new List<object>();
            foreach (var item in Children)
            {
                childrenList.Add(((TreeElement)item).ToTrentWithChildren());
            }

            var dct = new Dictionary<string, object>
            {
                { "type", type },
                { "name", ElementName },
                { "children", childrenList }
            };

            dct["pose"] = (object)
                new Dictionary<string, object>
                {
                    {
                        "position",
                        (object)
                            new List<object>
                            {
                                $"{local_pose.Lin.X:F3}",
                                $"{local_pose.Lin.Y:F3}",
                                $"{local_pose.Lin.Z:F3}"
                            }
                    },
                    {
                        "orientation",
                        (object)
                            new List<object>
                            {
                                $"{local_pose.Ang.X:F3}",
                                $"{local_pose.Ang.Y:F3}",
                                $"{local_pose.Ang.Z:F3}",
                                $"{local_pose.Ang.W:F3}"
                            }
                    }
                };

            if (type == "rotator" || type == "actuator")
            {
                var axis = AxisControl.Vector;
                dct["axis"] = new List<object> { axis.X, axis.Y, axis.Z };
            }

            dct["model"] = new Dictionary<string, object> { { "type", ModelType.ToLower() } };

            if (ModelType == "File" && !string.IsNullOrEmpty(ObjFilePath))
            {
                var modelDct = (Dictionary<string, object>)dct["model"];
                modelDct["path"] = ObjFilePath;
            }
            else if (ModelType == "Cube")
            {
                var modelDct = (Dictionary<string, object>)dct["model"];
                var size = CubeSizeControl.Vector;
                modelDct["size"] = new List<object> { size.X, size.Y, size.Z };
            }
            else if (ModelType == "Sphere")
            {
                var modelDct = (Dictionary<string, object>)dct["model"];
                modelDct["radius"] = SphereSizeControl.Radius;
            }
            else if (ModelType == "Cylinder")
            {
                var modelDct = (Dictionary<string, object>)dct["model"];
                modelDct["radius"] = CylinderSizeControl.Radius;
                modelDct["height"] = CylinderSizeControl.Height;
            }
            else if (ModelType == "Cone")
            {
                var modelDct = (Dictionary<string, object>)dct["model"];
                modelDct["radius1"] = ConeSizeControl.Radius1;
                modelDct["radius2"] = ConeSizeControl.Radius2;
                modelDct["height"] = ConeSizeControl.Height;
            }

            // if (type == "rotator")
            // {
            //     var rotator = (Rotator3)transform;
            //     dct["axis"] = new List<object>
            //     {
            //         rotator.Axis.X,
            //         rotator.Axis.Y,
            //         rotator.Axis.Z
            //     };
            // }
            // else if (type == "actuator")
            // {
            //     var actuator = (Actuator3)transform;
            //     dct["axis"] = new List<object>
            //     {
            //         actuator.Axis.X,
            //         actuator.Axis.Y,
            //         actuator.Axis.Z
            //     };
            // }

            return dct;
        }
    }

    public class SpartialToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is not SpatialTransformType type)
                return Visibility.Collapsed;

            return type == SpatialTransformType.Rotator || type == SpatialTransformType.Actuator
                ? Visibility.Visible
                : Visibility.Collapsed;
        }

        public object ConvertBack(
            object value,
            Type targetType,
            object parameter,
            CultureInfo culture
        )
        {
            throw new NotImplementedException();
        }
    }
}