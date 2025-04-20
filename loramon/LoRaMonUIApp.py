import urwid
import queue

class SubmitEdit(urwid.Edit):
    """Custom Edit widget that submits on Enter key."""
    def __init__(self, on_submit_callback, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.on_submit_callback = on_submit_callback

    def keypress(self, size, key):
        if key == 'enter':
            self.on_submit_callback()
            return None
        return super().keypress(size, key)


class LoRaMonUIApp:
    def __init__(self, queue_from_radio):

        #setup the Radio specific variables
        self.frequency = None
        self.bandwidth = None
        self.spread_factor = None
        self.coding_rate = None
        self.battery = None
        self.packets_received = 0

        #flag that indicates if the output should auto scroll to the bottom
        self.auto_scroll_flag = False

        # flag to select original or scrollable list
        self.ORIGINAL_WIDGET = False

        # Right pane output list
        self.output_lines = []
        if self.ORIGINAL_WIDGET: #original code
            self.output_widget = urwid.Text("", align="left")
            self.output_box = urwid.LineBox(
                urwid.Filler(self.output_widget, valign='top'),
                title="Output"
            )
        else: #scrollable
            self.output_widget = urwid.SimpleListWalker([])
            self.listbox = urwid.ListBox(self.output_widget)
            self.output_box = urwid.LineBox(self.listbox, title="Log Output")

        #list of widgets that get added to the left pane
        menu_widgets = []

        # Shared queue for sending messages
        self.message_queue = queue_from_radio

        #these are caption widgests, showing the paramters that comes from the radio
        self.caption_text_widgets = []
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio Freq: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   BW: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   SF: "), None))
        self.caption_text_widgets.append(urwid.AttrMap(urwid.Text("Radio   CR: "), None))

        #these are edit boxes, allowing the user to enter a value to be set in the radio
        self.input_edit_widgets = []
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self.submit_input,"Freq: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self.submit_input,"  BW: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self.submit_input,"  SF: ", ""), None))
        self.input_edit_widgets.append(urwid.AttrMap(SubmitEdit(self.submit_input,"  CR: ", ""), None))

        #add the captions and edit widges to the left menu
        for i in range(len(self.input_edit_widgets)):
            menu_widgets.append(self.caption_text_widgets[i])
            #don't added the edits, until they are implemented
            #menu_widgets.append(self.input_edit_widgets[i])

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

        # Left-top: Menu
        # these are 3 example buttons to put in. i'm using them as a template for other things
        # the first item, kicks off a thread to do background activity
        # second one is just a button
        menu_items = [("Menu 1", self.start_thread), ("Menu 2", self.menu_action)]

        for label, handler in menu_items:
            button = urwid.Button(label)
            urwid.connect_signal(button, 'click', handler, user_args={label})
            menu_widgets.append(urwid.AttrMap(button, None, focus_map='reversed'))

        menu_listbox = urwid.ListBox(urwid.SimpleFocusListWalker(menu_widgets))
        menu_box = urwid.LineBox(menu_listbox, title="Menu")

        # Left-bottom: Input
        # this area is meant to be a place for user to type commands
        # example quit/exit, bytes to send to the radio, etc.
        # nothing is implemented yet though
        self.input_edit = SubmitEdit(self.submit_input, caption="> ")
        submit_button = urwid.Button("Submit")
        urwid.connect_signal(submit_button, 'click', lambda button: self.submit_input())

        input_widgets = urwid.Pile([
            urwid.AttrMap(self.input_edit, None),
            urwid.AttrMap(submit_button, None, focus_map='reversed')
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

        self.loop = urwid.MainLoop(self.view, unhandled_input=self.handle_input)

        self.loop.set_alarm_in(.1, self.set_in_alarm_handler)

    def append_output(self, line):
        if self.ORIGINAL_WIDGET:
            self.output_lines.append(line)
            self.output_widget.set_text("\n".join(self.output_lines))
        else:
            self.output_widget.append(urwid.Text(line))
            # move to the bottom
            if (self.auto_scroll_flag == True):
                self.output_widget.set_focus(len(self.output_widget) - 1)

    def menu_action(self, button, label):
        self.append_output(f"You clicked: {label}")

    def start_thread(self, button, label):
        self.append_output("Menu 1: Starting background task...")
        thread = threading.Thread(target=self.background_task)
        thread.daemon = True
        thread.start()

    def toggleAutoScroll(self, button):
        if (self.auto_scroll_flag == True):
            self.auto_scroll_flag = False
        else:
            self.auto_scroll_flag = True
        self.auto_scroll_widget.set_label("AutoScroll: " + str(self.auto_scroll_flag))

    def background_task(self):
        time.sleep(.5)  # Simulate some background work
        #print("Background_task calling set_alarm_in")
        #self.loop.set_alarm_in(0, self.set_in_alarm_handler)
        msg = "<--->"
        #print(f"[Sender] Sending: {msg}\n")
        self.message_queue.put(msg)

    def set_in_alarm_handler(self, loop, data):
        #print("\nset_in_alarm_handler running")
        if not self.message_queue.empty():
            msg = self.message_queue.get()
            #print(f"[Receiver] Got message: {msg}")
            self.append_output(msg)

        #debug message, printing radio frequency
        #self.append_output(str(self.caption_text_widgets[0].original_widget.text))
        

        #update radio parameters
        self.caption_text_widgets[0].original_widget.set_text("Radio Freq: " + str(self.frequency))
        self.caption_text_widgets[1].original_widget.set_text("Radio   BW: " + str(self.bandwidth))
        self.caption_text_widgets[2].original_widget.set_text("Radio   SF: " + str(self.spread_factor))
        self.caption_text_widgets[3].original_widget.set_text("Radio   CR: " + str(self.coding_rate))
        self.battery_text_widget.original_widget.set_text    ("Battery: " + str(self.battery))
        self.packets_received_widget.original_widget.set_text    ("Packets: " + str(self.packet_received))

        self.loop.set_alarm_in(.1, self.set_in_alarm_handler)

    def submit_input(self):
        user_text = self.input_edit.edit_text.strip()
        if user_text:
            self.append_output(f"You entered: {user_text}")
            self.input_edit.set_edit_text("")  # Clear input

    def handle_input(self, key):
        #print("\n\nhandle_input")
        if key in ('q', 'Q'):
            raise urwid.ExitMainLoop()

    def run(self):
        self.loop.run()
