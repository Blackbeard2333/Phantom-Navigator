% Define Bus Elements
elems(1) = Simulink.BusElement;
elems(1).Name = 'Ixx';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Iyy';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Izz';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Im';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'mass';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'g';
elems(6).DataType = 'double';

% Create the Bus Object
Quad_Model_Bus = Simulink.Bus;
Quad_Model_Bus.Elements = elems;

% Assign to MATLAB base workspace
assignin('base', 'Quad_Model_Bus', Quad_Model_Bus);

