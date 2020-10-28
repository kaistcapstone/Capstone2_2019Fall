<?xml version='1.0' encoding='UTF-8'?>
<Library LVVersion="16008000">
	<Property Name="Alarm Database Computer" Type="Str">localhost</Property>
	<Property Name="Alarm Database Name" Type="Str">C__dev_robotics2010_dist_data</Property>
	<Property Name="Alarm Database Path" Type="Str">C:\dev\robotics2010\dist\data</Property>
	<Property Name="Data Lifespan" Type="UInt">3650</Property>
	<Property Name="Database Computer" Type="Str">localhost</Property>
	<Property Name="Database Name" Type="Str">C__dev_robotics2010_dist_data</Property>
	<Property Name="Database Path" Type="Str">C:\dev\robotics2010\dist\data</Property>
	<Property Name="Enable Alarms Logging" Type="Bool">true</Property>
	<Property Name="Enable Data Logging" Type="Bool">true</Property>
	<Property Name="Instrument Driver" Type="Str">True</Property>
	<Property Name="NI.Lib.Description" Type="Str">LabVIEW Plug and Play instrument driver for Dynamixel Motor.  Supported models include: DX113, DX117, RX10, RX28, RX64.</Property>
	<Property Name="NI.Lib.HelpPath" Type="Str"></Property>
	<Property Name="NI.Lib.Icon" Type="Bin">&amp;A#!!!!!!!)!"1!&amp;!!!-!%!!!@````]!!!!"!!%!!!*!!!!*Q(C=\&gt;;^&lt;2N"%-8R*]/"5X:A-X4[7G!,&lt;'%#OQ#G#NH#6'#!+M!"!Y&gt;+JA),&lt;)%FC0\@XIAC$%%+&lt;!%WI&amp;MN0^\OT@ZY8Z!UNA`3?ZX_?+03E`(&amp;BW&gt;8'?/D;63&lt;#J[T]`[0M][NS]\L^`D9`X,/\ZKH_-^M&lt;_.PY``X_-NX\`7,EVZZE[Y?\O'C*TXIHG`H)C`S)C`S)C`S*%`S*%`S*%`S)!`S)!`S)!^SEZP=Z#9XO2]?&amp;_1C&amp;\E](E(&amp;YM6#2&gt;'C1$%:CIK8QF.Y#E`BY;M+4_%J0)7H]$"&amp;B;@Q&amp;*\#5XD9497H]"3?QF.Y+$5E.2IZHM*$?37?R*.Y%E`C95EFHA31,*954IL!5$+9@%A]C3@R]&amp;'**`%EHM34?"B7YEE]C3@R*"ZW'5=FBW:KZ(AII]!4?!*0Y!E]F&amp;&lt;A#4S"*`!%(J:4Y!E]!3*9-#A/1=&amp;/Q94A3_!*0,QJ]!3?Q".Y!A^$YQT&amp;/$+4:GLE?)T(?)T(?)S(%D)?YT%?YT%?SMJYD-&gt;YD-&gt;Y7%L'9TT'9S"G5:;8+7:W.*.-9$T]D;P&amp;YSTFE(CUPD8H#V6^!;IP,05&amp;I\Y1V#&gt;9@?,5*U2^I.5(5(VAV$^9`505108#[I,KC4LS?K$P[4P[FL[BL_EL_J+_G(&lt;^SR/0R[-/BY0W_\VWOZWWW[UWGYX7[\67KZ77S[57C]8Z;@72.G^8F]_FW^0JRRXP0\^^OLH\MPN__`8TT4X@LZHX7HV_,PU,TU;^G`]P?DR'PQ!]:+O8!!!!!!</Property>
	<Property Name="NI.Lib.SourceVersion" Type="Int">369131520</Property>
	<Property Name="NI.Lib.Version" Type="Str">1.0.2.0</Property>
	<Property Name="NI.SortType" Type="Int">3</Property>
	<Property Name="SaveStatePeriod" Type="UInt">0</Property>
	<Property Name="Serialized ACL" Type="Bin">&amp;A#!!!!!!!)!"1!&amp;!!!A1%!!!@````]!!".V&lt;H.J:WZF:#"C?82F)'&amp;S=G&amp;Z!!%!!1!!!!A)!!!!#!!!!!!!!!!</Property>
	<Property Name="Use Data Logging Database" Type="Bool">true</Property>
	<Item Name="Public" Type="Folder">
		<Property Name="NI.LibItem.Scope" Type="Int">1</Property>
		<Item Name="Configure" Type="Folder">
			<Item Name="Advanced" Type="Folder">
				<Item Name="Configure Baud Rate.vi" Type="VI" URL="../Public/Configure/Advanced/Configure Baud Rate.vi"/>
				<Item Name="Configure Return Delay.vi" Type="VI" URL="../Public/Configure/Advanced/Configure Return Delay.vi"/>
				<Item Name="Configure PID.vi" Type="VI" URL="../Public/Configure/Advanced/Configure PID.vi"/>
			</Item>
			<Item Name="Active Compliance Settings.ctl" Type="VI" URL="../Public/Configure/Active Compliance Settings.ctl"/>
			<Item Name="Configure Safety Settings.vi" Type="VI" URL="../Public/Configure/Configure Safety Settings.vi"/>
			<Item Name="Configure Active Compliance.vi" Type="VI" URL="../Public/Configure/Configure Active Compliance.vi"/>
			<Item Name="Configure Resolution.vi" Type="VI" URL="../Public/Configure/Configure Resolution.vi"/>
			<Item Name="Configure Multi Turn Offset.vi" Type="VI" URL="../Public/Configure/Configure Multi Turn Offset.vi"/>
		</Item>
		<Item Name="Motor Operation" Type="Folder">
			<Item Name="Low Level" Type="Folder">
				<Item Name="Alarm Setting.ctl" Type="VI" URL="../Public/Motor Operation/Low Level/Alarm Setting.ctl"/>
				<Item Name="Generate Serial String.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Generate Serial String.vi"/>
				<Item Name="Read Address.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Read Address.vi"/>
				<Item Name="Read U8 From 1 Address.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Read U8 From 1 Address.vi"/>
				<Item Name="Read Word From 2 Addresses.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Read Word From 2 Addresses.vi"/>
				<Item Name="Send Direct Command.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Send Direct Command.vi"/>
				<Item Name="Write U8 To 1 Address.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Write U8 To 1 Address.vi"/>
				<Item Name="Write Word Over 2 Addresses.vi" Type="VI" URL="../Public/Motor Operation/Low Level/Write Word Over 2 Addresses.vi"/>
			</Item>
			<Item Name="Read Motor Position.vi" Type="VI" URL="../Public/Motor Operation/Read Motor Position.vi"/>
			<Item Name="Read Motor Velocity.vi" Type="VI" URL="../Public/Motor Operation/Read Motor Velocity.vi"/>
			<Item Name="Read Applied Torque.vi" Type="VI" URL="../Public/Motor Operation/Read Applied Torque.vi"/>
			<Item Name="Toggle Torque.vi" Type="VI" URL="../Public/Motor Operation/Toggle Torque.vi"/>
			<Item Name="Query Movement.vi" Type="VI" URL="../Public/Motor Operation/Query Movement.vi"/>
			<Item Name="Set Goal Accel.vi" Type="VI" URL="../Public/Motor Operation/Set Goal Accel.vi"/>
		</Item>
		<Item Name="Utility" Type="Folder">
			<Item Name="Change Motor ID.vi" Type="VI" URL="../Public/Utility/Change Motor ID.vi"/>
			<Item Name="Reset.vi" Type="VI" URL="../Public/Utility/Reset.vi"/>
			<Item Name="Query Temperature.vi" Type="VI" URL="../Public/Utility/Query Temperature.vi"/>
			<Item Name="Query Voltage.vi" Type="VI" URL="../Public/Utility/Query Voltage.vi"/>
			<Item Name="Query Lock.vi" Type="VI" URL="../Public/Utility/Query Lock.vi"/>
			<Item Name="Query Registered Instruction.vi" Type="VI" URL="../Public/Utility/Query Registered Instruction.vi"/>
			<Item Name="Scan for Motors.vi" Type="VI" URL="../Public/Utility/Scan for Motors.vi"/>
		</Item>
		<Item Name="Baud Setting.ctl" Type="VI" URL="../Public/Motor Operation/Low Level/Baud Setting.ctl"/>
	</Item>
	<Item Name="Private" Type="Folder">
		<Property Name="NI.LibItem.Scope" Type="Int">2</Property>
		<Item Name="Checksum.vi" Type="VI" URL="../Private/Checksum.vi"/>
		<Item Name="Process Motor Status.vi" Type="VI" URL="../Private/Process Motor Status.vi"/>
		<Item Name="Error Handler.vi" Type="VI" URL="../Private/Error Handler.vi"/>
	</Item>
</Library>
