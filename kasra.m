function varargout = kasra(varargin)
% KASRA MATLAB code for kasra.fig
%      KASRA, by itself, creates a new KASRA or raises the existing
%      singleton*.
%
%      H = KASRA returns the handle to a new KASRA or the handle to
%      the existing singleton*.
%
%      KASRA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KASRA.M with the given input arguments.
%
%      KASRA('Property','Value',...) creates a new KASRA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before kasra_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to kasra_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help kasra

% Last Modified by GUIDE v2.5 11-Jan-2021 17:10:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @kasra_OpeningFcn, ...
                   'gui_OutputFcn',  @kasra_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before kasra is made visible.
function kasra_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to kasra (see VARARGIN)


% Choose default command line output for kasra
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes kasra wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = kasra_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
chan =zeros(1,5);
save chan

function Fc_Callback(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fc as text
%        str2double(get(hObject,'String')) returns contents of Fc as a double


% --- Executes during object creation, after setting all properties.
function Fc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function time_Callback(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time as text
%        str2double(get(hObject,'String')) returns contents of time as a double


% --- Executes during object creation, after setting all properties.
function time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
    
end


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



t=[1:262144]*2*pi/262144;
for j=1:262144
    exp_vect(j)= exp(-1i*0*t(j)); %Construct freq. shift vector
end
 exp_vect =  exp_vect';
fc = get(handles.Fc,'string');
time = get(handles.time,'string'); 
time = str2double(time);
DB = get(handles.DB,'Value');
stop = get(handles.stop);
method =get(handles.method,'Value');
type =get(handles.type,'Value');
f_change =  get(handles.slider2,'Value');
var =0;
df = floor((f_change)*10^5);
fc = str2double(fc)+df;

tic
 FrontEndSampleRate = 250e3;
    FrameLength = 2^18;
     RTL_Obj= comm.SDRRTLReceiver(...
    'CenterFrequency',fc,...
    'EnableTunerAGC',true,...
    'SampleRate',FrontEndSampleRate,...
    'SamplesPerFrame',FrameLength,...
    'OutputDataType','double');
   d = fdesign.lowpass('N,Fc',30,8000,FrontEndSampleRate);
   Hd = design(d);
    d = fdesign.lowpass('N,Fc',40,10000,FrontEndSampleRate);
   Hd2 = design(d);
    d = fdesign.lowpass('N,Fc',30,100e3,FrontEndSampleRate);
   Hd3 = design(d);
    d = fdesign.lowpass('N,Fc',30,1,FrontEndSampleRate);
   Hd4 = design(d);
    d = fdesign.lowpass('N,Fc',50,25000,FrontEndSampleRate);
   Hd5 = design(d);
    d = fdesign.lowpass('N,Fc',30,100000,FrontEndSampleRate);
   Hd6 = design(d);
     d = fdesign.lowpass('N,Fc',1000,8000,FrontEndSampleRate);
   Hd7 = design(d);
        d = fdesign.lowpass('N,Fc',1000,25000,FrontEndSampleRate);
   Hd8 = design(d);
      d = fdesign.lowpass('N,Fc',1000,7000,FrontEndSampleRate);
   Hd9 = design(d);
   num = floor(10*FrontEndSampleRate/10);
   
   temp = zeros(num,1);
 
   i =1;
if method ==1
     while var <time
        data = step(RTL_Obj);
        Fs= 250e3;
        N0= length(data);
        N = length(data);
        Ts = 1/Fs;
        z=fftshift(fft(data)/Fs);%center noise spectrum
        f_vec=[0:1:N-1]*Fs/N-Fs/2; %designate sample frequencies
        amplitude_spectrum=abs(z); %compute two-sided amplitude spectrum
        ESD1=amplitude_spectrum.^2; %ESD = |F(w)|^2;
        PSD1=ESD1/((N0-1)*Ts);% PSD=ESD/T where T = total time of sample
        e = zeros(1,5);
        n = floor(length(PSD1)/5);
        e(1) = sum(PSD1(1:n));
        e(2) = sum(PSD1(n:2*n));
        e(3) = sum(PSD1(2*n:3*n));
        e(4) = sum(PSD1(3*n:4*n));
        e(5) = sum(PSD1(4*n:5*n));
        e = [e(1)/(e(1)+e(2)+e(3)+e(4)+e(5));e(2)/(e(1)+e(2)+e(3)+e(4)+e(5));e(3)/(e(1)+e(2)+e(3)+e(4)+e(5));...
            e(4)/(e(1)+e(2)+e(3)+e(4)+e(5));e(5)/(e(1)+e(2)+e(3)+e(4)+e(5))];
        maxe=find(e == max(e));
       
        if e(maxe) >.5
            fm = 1;
            fm_max = f_vec(n*(maxe));
            if maxe ==1
                fm_min = f_vec(1);
            else
                fm_min = f_vec(n*(maxe-1));
            end
        else
            fm =0;
        end
        if DB == 1
             plot(f_vec+fc,10*log10(PSD1)); 
             ylim([-250,-30]);
        else
            plot(f_vec+fc,abs(PSD1));
            ylim([0,.5*10^-3]);
        end
        xlabel('Frequency [Hz]','fontsize',14)
        ylabel('dB/Hz','fontsize',14)
        title('Power spectral density - method 1','fontsize',14)
        grid on;grid minor; 
        myString = sprintf('time : %1.1d s',var);
        set(handles.timer, 'String', myString);
        drawnow;
        var = toc;
         if (fm == 1) 
             myString = sprintf('We have an FM channel from %2.3e to  %2.3e Hz',fm_min+fc-Fs/2,fm_max+fc-Fs/2);
             set(handles.fm, 'String', myString);
        elseif (fm == 0) 
             myString = sprintf('there isnt an fm chanel');
             set(handles.fm, 'String', myString);
            
         end
         play = get(handles.palyradio,'Value');
         mute = get(handles.mute,'Value');
         if type == 1 && play ==1&& mute ==0
             %data = data.*exp_vect;
             s = get(handles.volume,'Value');
             s = floor((s)*5+0.5);
             x = filter(Hd3,data);
            % Normalize the signal;
            dataC = x;

            % Normalize the signal
            dataC_amplitude = abs(dataC);
            dataN = dataC./dataC_amplitude;

            % find the angle
            angleN = unwrap(angle(dataN));
            d_angleN = diff(angleN);       % FM demodulated data

            % scale the amplitude
            gain = 1/(2*pi);
            u_angleN1 = gain.* d_angleN;

            % low pass upto 8 KHz to keep data
            aud = filter(Hd,u_angleN1);
            a=1;
            % downsample from 250 KSPS to 25 KSPS
            aud1 = decimate(aud,  3);
            aud1 =s*filter(Hd7,aud1);
            sound((aud1),Fs/3)
         end
         if type == 2 && play ==1&& mute ==0
             %data = data.*exp_vect;
             s = get(handles.volume,'Value');
             s = floor((s)*3.5+0.5);
             t = 0:1/length(data):1/Fs;
             data = filter(Hd6,data);
             a = sign(real(data));
             for i = 2:length(a)
                 if a(i) == 0
                     out(i) =1;
                 elseif a(i) ==-1 &a(i-1)==1
                     out(i) = 1;
                 else
                     out(i) = 0;
                 end
             end

             aud = filter(Hd4,out);
             aud = filter(Hd7,aud);
             aud1 = decimate(aud, 3);
             play = filter(Hd7,aud1);
             sound(1.5*s*(play),Fs/3) 
         end
         if type == 3 && play ==1&& mute ==0
             s = get(handles.volume,'Value');
             s = (s)*.6+.5;
                      
             data = data ;
             %data = data .* exp(1j * 2*pi*100e3*t)';
             data_1 = data;
             a = diff(data_1);
             a = abs(hilbert(real(a)));
             a = decimate(a,2);
             a = filter(Hd9,a);
             sound(s*a,FrontEndSampleRate/2);
         end
     end
elseif method ==2
    while var <time
        data = step(RTL_Obj);
        Fs= 250e3;
        [pxx,f] = pwelch(data,2^18,0,50000,Fs);
        maxi=find(pxx == max(pxx));
        e = zeros(1,5);
        e(1) = sum(pxx(1:10000));
        e(2) = sum(pxx(10000:20000));
        e(3) = sum(pxx(20000:30000));
        e(4) = sum(pxx(30000:40000));
        e(5) = sum(pxx(40000:50000));
        e = [e(1)/(e(1)+e(2)+e(3)+e(4)+e(5));e(2)/(e(1)+e(2)+e(3)+e(4)+e(5));e(3)/(e(1)+e(2)+e(3)+e(4)+e(5));...
            e(4)/(e(1)+e(2)+e(3)+e(4)+e(5));e(5)/(e(1)+e(2)+e(3)+e(4)+e(5))];
        maxe=find(e == max(e));
        if e(maxe) >.5
            fm = 1;
            fm_max = f(10000*(maxe));
            fm_min = f(10000*(maxe-1));
        else
            fm =0;
        end
            
        if DB == 1
             plot(f+fc-Fs/2,10*log(pxx));
             hold on;
             plot(f(maxi)+fc-Fs/2,10*log(pxx(maxi)),'r*','MarkerSize' ,10,'LineWidth',3);
             hold off;
             ylim([-250,-40]);
        else
            plot(f+fc-Fs/2,pxx);
            hold on;
            plot(f(maxi)+fc-Fs/2,(pxx(maxi)),'r*','MarkerSize' ,10,'LineWidth',3);
            hold off;
            ylim([0,.5*10^-3]);
        end
        xlabel('Frequency [Hz]','fontsize',14)
        ylabel('dB/Hz','fontsize',14)
        title('Power spectral density - method 2 - pwelch','fontsize',14)
        grid on;grid minor; 
        if (fm == 1) 
             myString = sprintf('We have an FM channel from %2.3e to  %2.3e Hz',fm_min+fc-Fs/2,fm_max+fc-Fs/2);
             set(handles.fm, 'String', myString);
        elseif (fm == 0) 
             myString = sprintf('there isnt an fm chanel');
             set(handles.fm, 'String', myString);
            
        end
        drawnow;
        myString = sprintf('time : %1.1d s',var);
        set(handles.timer, 'String', myString);
        var = toc; 
       play = get(handles.palyradio,'Value');
         mute = get(handles.mute,'Value');
         if type == 1 && play ==1&& mute ==0

             s = get(handles.volume,'Value');
             s = floor((s)*5+0.5);
             x = filter(Hd3,data);
            % Normalize the signal;
            dataC = x;

            % Normalize the signal
            dataC_amplitude = abs(dataC);
            dataN = dataC./dataC_amplitude;

            % find the angle
            angleN = unwrap(angle(dataN));
            d_angleN = diff(angleN);       % FM demodulated data

            % scale the amplitude
            gain = 1/(2*pi);
            u_angleN1 = gain.* d_angleN;

            % low pass upto 8 KHz to keep data
            aud = filter(Hd,u_angleN1);
            a=1;
            % downsample from 250 KSPS to 25 KSPS
            aud1 = decimate(aud,  3);
            aud1 =s*filter(Hd7,aud1);
            sound((aud1),Fs/3)
         end
         if type == 2 && play ==1&& mute ==0
             s = get(handles.volume,'Value');
             s = floor((s)*3.5+0.5);
             data = filter(Hd6,data);
             a = sign(real(data));
             for i = 2:length(a)
                 if a(i) == 0
                     out(i) =1;
                 elseif a(i) ==-1 &a(i-1)==1
                     out(i) = 1;
                 else
                     out(i) = 0;
                 end
             end

             aud = filter(Hd4,out);
             aud1 = decimate(aud, 3);
             play = filter(Hd7,aud1);
             sound(1.5*s*(play),Fs/3)

         end
         if type == 3 && play ==1&& mute ==0
             s = get(handles.volume,'Value');
             s = (s)*.6+.5;
                      
             data = data ;
             %data = data .* exp(1j * 2*pi*100e3*t)';
             data_1 = data;
             a = diff(data_1);
             a= abs(hilbert(real(a)));
             a = decimate(a,2);
             a = filter(Hd9,a);
             sound(s*a,FrontEndSampleRate/2);
         end
        
        
    end
           
    
end
% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    %set background color from grey (default) to white
  % periodogram(data,rectwin(length(data)),length(data),Fs)
   
 
% Hint: place code in OpeningFcn to populate axes1


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in DB.
function DB_Callback(hObject, eventdata, handles)
% hObject    handle to DB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of DB


% --- Executes during object creation, after setting all properties.
function DB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function stop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on selection change in method.
function method_Callback(hObject, eventdata, handles)
% hObject    handle to method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns method contents as cell array
%        contents{get(hObject,'Value')} returns selected item from method


% --- Executes during object creation, after setting all properties.
function method_CreateFcn(hObject, eventdata, handles)
% hObject    handle to method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function fm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function timer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function volume_Callback(hObject, eventdata, handles)
% hObject    handle to volume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function volume_CreateFcn(hObject, eventdata, handles)
% hObject    handle to volume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in playchannel.
function playchannel_Callback(hObject, eventdata, handles)
% hObject    handle to playchannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of playchannel


% --- Executes during object creation, after setting all properties.
function playchannel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to playchannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in mute.
function mute_Callback(hObject, eventdata, handles)
% hObject    handle to mute (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mute


% --- Executes during object creation, after setting all properties.
function mute_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mute (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object deletion, before destroying properties.
function playchannel_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to playchannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in palyradio.
function palyradio_Callback(hObject, eventdata, handles)
% hObject    handle to palyradio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of palyradio


% --- Executes during object creation, after setting all properties.
function palyradio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to palyradio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on selection change in type.
function type_Callback(hObject, eventdata, handles)
% hObject    handle to type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns type contents as cell array
%        contents{get(hObject,'Value')} returns selected item from type


% --- Executes during object creation, after setting all properties.
function type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in channels.
function channels_Callback(hObject, eventdata, handles)
% hObject    handle to channels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns channels contents as cell array
%        contents{get(hObject,'Value')} returns selected item from channels


% --- Executes during object creation, after setting all properties.
function channels_CreateFcn(hObject, eventdata, handles)
% hObject    handle to channels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num =get(handles.channels,'Value');
fc = get(handles.Fc,'string');
f_change =  get(handles.slider2,'Value');
df = floor((f_change)*10^5);
fc = str2double(fc)+df;
X = load('chan');
temp = X.chan ;
temp(num) = fc;
chan = temp;
save chan


% --- Executes on button press in palychannel.
function palychannel_Callback(hObject, eventdata, handles)
% hObject    handle to palychannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num =get(handles.channels,'Value');
X = load('chan');
temp = X.chan 
fc = temp(num) ;
time = get(handles.time,'string'); 
time = str2double(time);
DB = get(handles.DB,'Value');
stop = get(handles.stop);
method =get(handles.method,'Value');
type =get(handles.type,'Value');
var = 0;
tic
 FrontEndSampleRate = 250e3;
    FrameLength = 2^18;
     RTL_Obj= comm.SDRRTLReceiver(...
    'CenterFrequency',fc,...
    'EnableTunerAGC',true,...
    'SampleRate',FrontEndSampleRate,...
    'SamplesPerFrame',FrameLength,...
    'OutputDataType','double');
   d = fdesign.lowpass('N,Fc',30,8000,FrontEndSampleRate);
   Hd = design(d);
    d = fdesign.lowpass('N,Fc',40,10000,FrontEndSampleRate);
   Hd2 = design(d);
    d = fdesign.lowpass('N,Fc',30,100e3,FrontEndSampleRate);
   Hd3 = design(d);
    d = fdesign.lowpass('N,Fc',30,1,FrontEndSampleRate);
   Hd4 = design(d);
    d = fdesign.lowpass('N,Fc',50,25000,FrontEndSampleRate);
   Hd5 = design(d);
    d = fdesign.lowpass('N,Fc',30,100000,FrontEndSampleRate);
   Hd6 = design(d);
     d = fdesign.lowpass('N,Fc',1000,8000,FrontEndSampleRate);
   Hd7 = design(d);
   num = floor(10*FrontEndSampleRate/10);
   
   temp = zeros(num,1);
 
   i =1;
if method ==1
     while var <time
        data = step(RTL_Obj);
        Fs= 250e3;
        N0= length(data);
        N = length(data);
        Ts = 1/Fs;
        z=fftshift(fft(data)/Fs);%center noise spectrum
        f_vec=[0:1:N-1]*Fs/N-Fs/2; %designate sample frequencies
        amplitude_spectrum=abs(z); %compute two-sided amplitude spectrum
        ESD1=amplitude_spectrum.^2; %ESD = |F(w)|^2;
        PSD1=ESD1/((N0-1)*Ts);% PSD=ESD/T where T = total time of sample
        e = zeros(1,5);
        n = floor(length(PSD1)/5);
        e(1) = sum(PSD1(1:n));
        e(2) = sum(PSD1(n:2*n));
        e(3) = sum(PSD1(2*n:3*n));
        e(4) = sum(PSD1(3*n:4*n));
        e(5) = sum(PSD1(4*n:5*n));
        e = [e(1)/(e(1)+e(2)+e(3)+e(4)+e(5));e(2)/(e(1)+e(2)+e(3)+e(4)+e(5));e(3)/(e(1)+e(2)+e(3)+e(4)+e(5));...
            e(4)/(e(1)+e(2)+e(3)+e(4)+e(5));e(5)/(e(1)+e(2)+e(3)+e(4)+e(5))];
        maxe=find(e == max(e));
       
        if e(maxe) >.5
            fm = 1;
            fm_max = f_vec(n*(maxe));
            if maxe ==1
                fm_min = f_vec(1);
            else
                fm_min = f_vec(n*(maxe-1));
            end
        else
            fm =0;
        end
        if DB == 1
             plot(f_vec+fc,10*log10(PSD1)); 
             ylim([-250,-30]);
        else
            plot(f_vec+fc,abs(PSD1));
            ylim([0,.5*10^-3]);
        end
        xlabel('Frequency [Hz]','fontsize',14)
        ylabel('dB/Hz','fontsize',14)
        title('Power spectral density - method 1','fontsize',14)
        grid on;grid minor; 
        myString = sprintf('time : %1.1d s',var);
        set(handles.timer, 'String', myString);
        drawnow;
        var = toc;
         if (fm == 1) 
             myString = sprintf('We have an FM channel from %2.3e to  %2.3e Hz',fm_min+fc-Fs/2,fm_max+fc-Fs/2);
             set(handles.fm, 'String', myString);
        elseif (fm == 0) 
             myString = sprintf('there isnt an fm chanel');
             set(handles.fm, 'String', myString);
            
         end
         play = get(handles.palyradio,'Value');
         mute = get(handles.mute,'Value');
         if type == 1 && play ==1&& mute ==0
             %data = data.*exp_vect;
             s = get(handles.volume,'Value');
             s = floor((s)*5+0.5);
             x = filter(Hd3,data);
            % Normalize the signal;
            dataC = x;

            % Normalize the signal
            dataC_amplitude = abs(dataC);
            dataN = dataC./dataC_amplitude;

            % find the angle
            angleN = unwrap(angle(dataN));
            d_angleN = diff(angleN);       % FM demodulated data

            % scale the amplitude
            gain = 1/(2*pi);
            u_angleN1 = gain.* d_angleN;

            % low pass upto 8 KHz to keep data
            aud = filter(Hd,u_angleN1);
            a=1;
            % downsample from 250 KSPS to 25 KSPS
            aud1 = decimate(aud,  3);
            aud1 =s*filter(Hd7,aud1);
            sound((aud1),Fs/3)
         end
         if type == 2 && play ==1&& mute ==0
             %data = data.*exp_vect;
             s = get(handles.volume,'Value');
             s = floor((s)*3.5+0.5);
             t = 0:1/length(data):1/Fs;
             data = filter(Hd6,data);
             a = sign(real(data));
             for i = 2:length(a)
                 if a(i) == 0
                     out(i) =1;
                 elseif a(i) ==-1 &a(i-1)==1
                     out(i) = 1;
                 else
                     out(i) = 0;
                 end
             end

             aud = filter(Hd4,out);
             aud = filter(Hd7,aud);
             aud1 = decimate(aud, 5);
             play = filter(Hd7,aud1);
             sound(s*(play),Fs/5) 
         end
     end
elseif method ==2
    while var <time
        data = step(RTL_Obj);
        Fs= 250e3;
        [pxx,f] = pwelch(data,2^18,0,50000,Fs);
        maxi=find(pxx == max(pxx));
        e = zeros(1,5);
        e(1) = sum(pxx(1:10000));
        e(2) = sum(pxx(10000:20000));
        e(3) = sum(pxx(20000:30000));
        e(4) = sum(pxx(30000:40000));
        e(5) = sum(pxx(40000:50000));
        e = [e(1)/(e(1)+e(2)+e(3)+e(4)+e(5));e(2)/(e(1)+e(2)+e(3)+e(4)+e(5));e(3)/(e(1)+e(2)+e(3)+e(4)+e(5));...
            e(4)/(e(1)+e(2)+e(3)+e(4)+e(5));e(5)/(e(1)+e(2)+e(3)+e(4)+e(5))];
        maxe=find(e == max(e));
        if e(maxe) >.5
            fm = 1;
            fm_max = f(10000*(maxe));
            fm_min = f(10000*(maxe-1));
        else
            fm =0;
        end
            
        if DB == 1
             plot(f+fc-Fs/2,10*log(pxx));
             hold on;
             plot(f(maxi)+fc-Fs/2,10*log(pxx(maxi)),'r*','MarkerSize' ,10,'LineWidth',3);
             hold off;
             ylim([-250,-40]);
        else
            plot(f+fc-Fs/2,pxx);
            hold on;
            plot(f(maxi)+fc-Fs/2,(pxx(maxi)),'r*','MarkerSize' ,10,'LineWidth',3);
            hold off;
            ylim([0,.5*10^-3]);
        end
        xlabel('Frequency [Hz]','fontsize',14)
        ylabel('dB/Hz','fontsize',14)
        title('Power spectral density - method 2 - pwelch','fontsize',14)
        grid on;grid minor; 
        if (fm == 1) 
             myString = sprintf('We have an FM channel from %2.3e to  %2.3e Hz',fm_min+fc-Fs/2,fm_max+fc-Fs/2);
             set(handles.fm, 'String', myString);
        elseif (fm == 0) 
             myString = sprintf('there isnt an fm chanel');
             set(handles.fm, 'String', myString);
            
        end
        drawnow;
        myString = sprintf('time : %1.1d s',var);
        set(handles.timer, 'String', myString);
        var = toc; 
       play = get(handles.palyradio,'Value');
         mute = get(handles.mute,'Value');
         if type == 1 && play ==1&& mute ==0
             
             s = get(handles.volume,'Value');
             s = floor((s)*5+0.5);
             x = filter(Hd3,data);
            % Normalize the signal;
            dataC = x;

            % Normalize the signal
            dataC_amplitude = abs(dataC);
            dataN = dataC./dataC_amplitude;

            % find the angle
            angleN = unwrap(angle(dataN));
            d_angleN = diff(angleN);       % FM demodulated data

            % scale the amplitude
            gain = 1/(2*pi);
            u_angleN1 = gain.* d_angleN;

            % low pass upto 8 KHz to keep data
            aud = filter(Hd,u_angleN1);
            a=1;
            % downsample from 250 KSPS to 25 KSPS
            aud1 = decimate(aud,  8);
            aud1 =s*filter(Hd2,aud1);
            sound((aud1),Fs/8)
         end
         if type == 2 && play ==1&& mute ==0
             s = get(handles.volume,'Value');
             s = floor((s)*3.5+0.5);
             data = filter(Hd6,data);
             a = sign(real(data));
             for i = 2:length(a)
                 if a(i) == 0
                     out(i) =1;
                 elseif a(i) ==-1 &a(i-1)==1
                     out(i) = 1;
                 else
                     out(i) = 0;
                 end
             end

             aud = filter(Hd4,out);
             aud1 = decimate(aud, 3);
             play = filter(Hd7,aud1);
             sound(s*(play),Fs/3) 
         end

        
    end
end
