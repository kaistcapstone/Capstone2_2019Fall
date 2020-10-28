<?xml version='1.0' encoding='UTF-8'?>
<Project Name="Template - Generic.lvproj" Type="Project" LVVersion="16008000" URL="/&lt;instrlib&gt;/_Template - Generic/Template - Generic.lvproj">
	<Property Name="Instrument Driver" Type="Str">True</Property>
	<Property Name="NI.Project.Description" Type="Str">LabVIEW Plug and Play sensor driver and example for the Dynamixel Motor Series. For more information about this driver, please refer to Dynamixel Motor Readme.html.</Property>
	<Property Name="varPersistentID:{203B2AC7-167A-4FFE-BF7D-5129A86033E0}" Type="Ref">/My Computer/Examples/Dynamixel Motor Record and Playback Support/Globals.lvlib/Copied Motions</Property>
	<Property Name="varPersistentID:{3D082006-7FFF-49D5-9802-E258E595BF5B}" Type="Ref">/My Computer/Examples/Dynamixel Motor Record and Playback Support/Globals.lvlib/Stop Playback</Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="CCSymbols" Type="Str">OS,Win;CPU,x86;</Property>
		<Property Name="IOScan.Faults" Type="Str"></Property>
		<Property Name="IOScan.NetVarPeriod" Type="UInt">100</Property>
		<Property Name="IOScan.Period" Type="UInt">10000</Property>
		<Property Name="IOScan.PowerupMode" Type="UInt">0</Property>
		<Property Name="IOScan.Priority" Type="UInt">9</Property>
		<Property Name="IOScan.ReportModeConflict" Type="Bool">false</Property>
		<Property Name="IOScan.StartEngineOnDeploy" Type="Bool">false</Property>
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Examples" Type="Folder">
			<Item Name="Dynamixel Motor Record and Playback Support" Type="Folder">
				<Item Name="Sub VIs" Type="Folder">
					<Item Name="ASCII Positions and Times to Dynamixel Positions and Speeds.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/ASCII Positions and Times to Dynamixel Positions and Speeds.vi"/>
					<Item Name="Capture Motor Positions.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/Capture Motor Positions.vi"/>
					<Item Name="Generate Synchronous Strings For Moving Motors.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/Generate Synchronous Strings For Moving Motors.vi"/>
					<Item Name="Open Motion.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/Open Motion.vi"/>
					<Item Name="Play Motion.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/Play Motion.vi"/>
					<Item Name="Save Motion.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Sub VIs/Save Motion.vi"/>
				</Item>
				<Item Name="Type Defs" Type="Folder">
					<Item Name="Compiled Motion.ctl" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Type Defs/Compiled Motion.ctl"/>
					<Item Name="Raw Motion.ctl" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Type Defs/Raw Motion.ctl"/>
				</Item>
				<Item Name="Dynamixel Readme.html" Type="Document" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Documents/Dynamixel Readme.html"/>
				<Item Name="Globals.lvlib" Type="Library" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Globals.lvlib"/>
			</Item>
			<Item Name="Dyanmixel Motor Read All Data From Motor(s).vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Dyanmixel Motor Read All Data From Motor(s).vi"/>
			<Item Name="Dynamixel Motor Record and Playback Motion (Synchronous Write).vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Record And Playback Motion/Dynamixel Motor Record and Playback Motion (Synchronous Write).vi"/>
			<Item Name="Dynamixel Motor Send Advanced Commands.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Dynamixel Motor Send Advanced Commands.vi"/>
			<Item Name="Dynamixel Motor Setup.vi" Type="VI" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Dynamixel Motor Setup.vi"/>
			<Item Name="Dynamixel Motor.bin3" Type="Document" URL="/&lt;instrlib&gt;/Dynamixel Motor/Examples/Dynamixel Motor.bin3"/>
		</Item>
		<Item Name="Documents" Type="Folder">
			<Item Name="RX-10_Manual.pdf" Type="Document" URL="/&lt;instrlib&gt;/Dynamixel Motor/Docs/RX-10_Manual.pdf"/>
			<Item Name="RX-28_Manual.pdf" Type="Document" URL="/&lt;instrlib&gt;/Dynamixel Motor/Docs/RX-28_Manual.pdf"/>
			<Item Name="RX-64_Manual.pdf" Type="Document" URL="/&lt;instrlib&gt;/Dynamixel Motor/Docs/RX-64_Manual.pdf"/>
		</Item>
		<Item Name="Dynamixel Motor.lvlib" Type="Library" URL="/&lt;instrlib&gt;/Dynamixel Motor/Dynamixel Motor.lvlib"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="VISA Configure Serial Port" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port"/>
				<Item Name="VISA Configure Serial Port (Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Instr).vi"/>
				<Item Name="VISA Configure Serial Port (Serial Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Serial Instr).vi"/>
				<Item Name="subFile Dialog.vi" Type="VI" URL="/&lt;vilib&gt;/express/express input/FileDialogBlock.llb/subFile Dialog.vi"/>
				<Item Name="ex_CorrectErrorChain.vi" Type="VI" URL="/&lt;vilib&gt;/express/express shared/ex_CorrectErrorChain.vi"/>
			</Item>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
