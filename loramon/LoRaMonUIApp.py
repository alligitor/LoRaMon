import urwid
import queue
import time
import threading

class SubmitEdit(urwid.Edit):
    """Custom Edit widget that submits on Enter key."""
    def __init__(self, parentApp, name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parentApp = parentApp
        self.name = name

    def keypress(self, size, key):
        if key == 'enter':
            self.submitHandler(self.parentApp)
            return None
        return super().keypress(size, key)

    def submitHandler(self, parentApp):
        parentApp.widgetChangeHandler(self.name, self.edit_text.strip())

class LoRaMonUIApp:
    def __init__(self, queue_from_radio, queue_to_radio):

        #flag that indicates if the output should auto scroll to the bottom
        self.auto_scroll_flag = True

        #flag that indicates if the output should auto scroll to the bottom
        self.print_raw_data_flag = False

        # Right pane output list
        self.output_lines = []
        #scrollable
        self.output_widget = urwid.SimpleListWalker([])
        self.listbox = urwid.ListBox(self.output_widget)
        self.output_box = urwid.LineBox(self.listbox, title="Output - Hit Q to Quit")

        #list of widgets that get added to the left pane
        menu_widgets = []

        # Shared queue for receiving updates from the radio
        self.queue_from_radio = queue_from_radio
        # Shared queue for sending messages to radio
        self.queue_to_radio = queue_to_radio

        #these are caption widgests, showing the paramters that comes from the radio
        self.caption_text_widgets = []
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio Freq: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   BW: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   SF: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   CR: "), None))

        #these are edit boxes, allowing the user to enter a value to be set in the radio
        self.input_edit_widgets = []
        #                                            Widget    UI App   Name       Caption   initial val
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self, "frequency"    , "Freq: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self, "bandwidth"    , "  BW: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self, "spread_factor", "  SF: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self, "coding_rate"  , "  CR: ", ""), None))

        #add the captions and edit widges to the left menu
        for i in range(len(self.input_edit_widgets)):
            menu_widgets.append(self.caption_text_widgets[i])
            #don't added the edits, until they are implemented
            menu_widgets.append(self.input_edit_widgets[i])

        #widget for showing battery status
        self.battery_text_widget = urwid.AttrMap(urwid.Text("Battery: "), None)
        menu_widgets.append(self.battery_text_widget)

        #widget for showing number of packets that have been captured
        self.packets_received_widget = urwid.AttrMap(urwid.Text("Packets: "), None)
        menu_widgets.append(self.packets_received_widget)

        #widget for turn auto scroll on / off
        self.auto_scroll_widget = urwid.Button("AutoScroll: " + str(self.auto_scroll_flag))
        urwid.connect_signal(self.auto_scroll_widget, 'click', self.toggleAutoScroll)
        menu_widgets.append(self.auto_scroll_widget)

        #widget for turn auto scroll on / off
        self.print_raw_data_widget = urwid.Button("Print Raw Data: " + str(self.print_raw_data_flag))
        urwid.connect_signal(self.print_raw_data_widget, 'click', self.togglePrintRawData)
        menu_widgets.append(self.print_raw_data_widget)

        # Left-top: Menu
        # these are 2 example buttons to put in. i'm using them as a template for other things
        # the first item, kicks off a thread to do background activity
        # second one is just a button
        menu_items = [("Menu 1", self.start_thread), ("Menu 2", self.menu_action)]

        for label, handler in menu_items:
            button = urwid.Button(label)
            urwid.connect_signal(button, 'click', handler, user_args={label})
            menu_widgets.append(urwid.AttrMap(button, None, focus_map='reversed'))

        menu_listbox = urwid.ListBox(urwid.SimpleFocusListWalker(menu_widgets))
        menu_box = urwid.LineBox(menu_listbox, title="Controls")

        # Left-bottom: Input
        # this area is meant to be a place for user to type commands
        # example quit/exit, bytes to send to the radio, etc.
        # nothing is implemented yet though
        self.user_command_widget = SubmitEdit(self, "user_command", caption="Cmd: ")

        quit_button = urwid.Button("Quit")
        urwid.connect_signal(quit_button, 'click', lambda button: self.quitApp())

        input_widgets = urwid.Pile([
            urwid.AttrMap(self.user_command_widget, None),
            urwid.AttrMap(quit_button, None, focus_map='reversed')
        ])
        input_box = urwid.LineBox(input_widgets, title="User Input")

        # Left Pane: Stack menu + input
        left_pane = urwid.Pile([
            ('weight', 2, menu_box),
            ('weight', 1, input_box)
        ])

        # Layout: Columns
        columns = urwid.Columns([
            ('weight', 1, left_pane),
            ('weight', 2, self.output_box)
        ])

        if False:
            self.view = urwid.Frame(
                header=urwid.Text("Menu Thread Example — Press 'q' to quit"),
                body=columns
            )
        else:
            self.view = columns

        self.loop = urwid.MainLoop(self.view, unhandled_input=self.unhandledInputHandler)

        self.loop.set_alarm_in(.1, self.radioMessageHandler)

    def sendParameterToRadio(self, type, value):
        if (self.queue_to_radio != None):
            msg = {
                "type": type,
                "value": value
                }
            self.queue_to_radio.put(msg)
                
    def widgetChangeHandler(self, widgetName, value):
        def to_integer(s):
            try:
                return int(s)
            except ValueError:
                return None

        #most values are integers, convert to int here
        valueInt = to_integer(value)

        match(widgetName):
            case "frequency":
                #self.appendToOutputWidget(f"Widget {widgetName} received {value}")
                if valueInt != None:
                    #we should do a check here for valid freq ranges..
                    self.sendParameterToRadio(widgetName, valueInt)
            case "bandwidth":
                #self.appendToOutputWidget(f"Widget {widgetName} received {value}")
                if valueInt != None:
                    #only support 125k and 250K values
                    if valueInt == 125000 or valueInt == 250000:
                        self.sendParameterToRadio(widgetName, valueInt)
            case "spread_factor":
                #self.appendToOutputWidget(f"Widget {widgetName} received {value}")
                if valueInt != None:
                    #support spread factors 7 .. 12
                    if valueInt >= 7 and valueInt <= 12:
                        self.sendParameterToRadio(widgetName, valueInt)
            case "coding_rate":
                #self.appendToOutputWidget(f"Widget {widgetName} received {value}")
                if valueInt != None:
                    #only support coding rages 5 .. 8
                    if valueInt >= 5 and valueInt <= 8:
                        self.sendParameterToRadio(widgetName, valueInt)
            case "user_command":
                #handle the widget that handles generic commands
                #right now, nothing is implemented, just print a message to show what was received.
                self.appendToOutputWidget(f"Widget {widgetName} received {value}")
            case _:
                None

    def appendToOutputWidget(self, line):
        self.output_widget.append(urwid.Text(line))
        # move to the bottom
        if (self.auto_scroll_flag == True):
            self.output_widget.set_focus(len(self.output_widget) - 1)

    def menu_action(self, button, label):
        self.appendToOutputWidget(f"You clicked: {label}")

    def toggleAutoScroll(self, button):
        if (self.auto_scroll_flag == True):
            self.auto_scroll_flag = False
        else:
            self.auto_scroll_flag = True
        self.auto_scroll_widget.set_label("AutoScroll: " + str(self.auto_scroll_flag))

    def togglePrintRawData(self, button):
        if (self.print_raw_data_flag == True):
            self.print_raw_data_flag = False
        else:
            self.print_raw_data_flag = True
        self.print_raw_data_widget.set_label("Print Raw Data: " + str(self.print_raw_data_flag))
        #tell the radio the status of the button
        self.sendParameterToRadio("print_raw_data", self.print_raw_data_flag)

    #this routine is called periodicaly to service incoming events for the UI
    #currently, there is a message queue from the radio thread
    #that send messages to be shown
    def radioMessageHandler(self, loop, data):
        #print("\nradioMessageHandler running")
        # read up to 10 messages at a time
        num_messages = 10

        while not self.queue_from_radio.empty() and num_messages > 0:
            num_messages -= 1

            msg = self.queue_from_radio.get()
            match msg['type']:
                case "LogMessageFromRadio":
                    self.appendToOutputWidget(msg["value"])
                case "r_frequency":
                    self.caption_text_widgets[0].original_widget.set_text("Radio Freq: " + str(msg["value"]))
                case "r_bandwidth":
                    self.caption_text_widgets[1].original_widget.set_text("Radio   BW: " + str(msg["value"]))
                case "r_spread_factor":
                    self.caption_text_widgets[2].original_widget.set_text("Radio   SF: " + str(msg["value"]))
                case "r_coding_rate":
                    self.caption_text_widgets[3].original_widget.set_text("Radio   CR: " + str(msg["value"]))
                case "r_battery":
                    self.battery_text_widget.original_widget.set_text    ("Battery: " + str(msg["value"]))
                case "r_captured_packets":
                    self.packets_received_widget.original_widget.set_text("Packets: " + str(msg["value"]))
                case "print_raw_data":
                    self.print_raw_data_flag = msg["value"]
                    self.print_raw_data_widget.set_label("Print Raw Data: " + str(self.print_raw_data_flag))
                case _:
                    None
        #set the alarm again, so it calls the routing again
        self.loop.set_alarm_in(.1, self.radioMessageHandler)

    def quitApp(self):
        raise urwid.ExitMainLoop()

    def unhandledInputHandler(self, key):
        #print("\n\nunhandledInputHandler")
        if key in ('q', 'Q'):
            raise urwid.ExitMainLoop()

    def run(self):
        self.loop.run()

    # these two functions were for starting a new thread to do some background work
    # after menu 1 was hit.  Not sure if they are needed but keeping them here in case
    def start_thread(self, button, label):
        self.appendToOutputWidget("Menu 1: Starting background task...")
        thread = threading.Thread(target=self.background_task)
        thread.daemon = True
        thread.start()

    def background_task(self):
        time.sleep(.5)  # Simulate some background work
        self.appendToOutputWidget("Menu 1: background_task finished")

