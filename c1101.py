# cc1101_oook_decoder_learning.py
# MicroPython for Raspberry Pi Pico
#
# Features:
#  - Auto SHORT/LONG pulse estimation
#  - Auto inversion detection
#  - Frame repeat filtering per device
#  - Multi-device recognition + learning unknown sensors
#  - Starts with known Kerui door sensor

from machine import Pin, SPI
import time
import array

# ----------------- USER TUNABLES -----------------
SAMPLE_MAX_EDGES = 4000
SYNC_GAP_US = 5000
# -----------------------------------------------------------------------

# ----------------- SPI + CC1101 -----------------
spi = SPI(0, baudrate=2000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs = Pin(5, Pin.OUT, value=1)

def cs_low(): cs.value(0)
def cs_high(): cs.value(1)
def strobe(cmd): cs_low(); spi.write(bytearray([cmd])); cs_high()
def write_reg(addr,val): cs_low(); spi.write(bytearray([addr,val])); cs_high()
def read_reg(addr): cs_low(); spi.write(bytearray([addr|0x80])); b=spi.read(1); cs_high(); return b[0]

def reset_cc1101():
    cs_high(); time.sleep_us(30); cs_low(); time.sleep_us(30)
    cs_high(); time.sleep_us(45)
    strobe(0x30); time.sleep_ms(1)

def init_cc1101():
    reset_cc1101()
    regs = {
        0x00:0x0B, 0x02:0x06, 0x0B:0x0C, 0x0C:0x00,
        0x0D:0x10,0x0E:0xB0,0x0F:0x71,0x10:0x2D,
        0x11:0x3B,0x12:0x73,0x15:0x34,0x18:0x18,
        0x19:0x16,0x1B:0x43,0x21:0x56,0x23:0xE9,
        0x24:0x2A,0x25:0x00,0x26:0x1F,0x29:0x59,
        0x2C:0x81,0x2D:0x35,0x2E:0x09
    }
    for a,v in regs.items(): write_reg(a,v)
    strobe(0x34)  # SRX
    print("CC1101 initialized and set to RX mode")

# ----------------- Edge capture -----------------
EDGE_PIN=6
data_pin=Pin(EDGE_PIN,Pin.IN)
times=array.array('I',[0])*SAMPLE_MAX_EDGES
levels=array.array('b',[0])*SAMPLE_MAX_EDGES
edge_idx=0
capturing=False

def irq_handler(pin):
    global edge_idx,capturing
    if not capturing: return
    if edge_idx>=SAMPLE_MAX_EDGES:
        capturing=False
        pin.irq(handler=None)
        return
    times[edge_idx]=time.ticks_us()
    levels[edge_idx]=pin.value()
    edge_idx+=1

def start_capture(max_edges=SAMPLE_MAX_EDGES):
    global edge_idx,capturing
    edge_idx=0
    capturing=True
    _=data_pin.value()
    data_pin.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler=irq_handler)
    print("Capture started (max edges={})".format(max_edges))

def stop_capture():
    global capturing
    capturing=False
    data_pin.irq(handler=None)
    print("Capture stopped")

# ----------------- Utilities -----------------
def get_runs():
    if edge_idx<2: return []
    runs=[]
    prev_t=times[0]
    prev_level=levels[0]
    for i in range(1,edge_idx):
        t=times[i]; lvl=levels[i]
        dur=time.ticks_diff(t,prev_t)
        runs.append((dur,prev_level))
        prev_t=t; prev_level=lvl
    return runs

def split_frames(runs):
    frames=[]
    cur=[]
    for dur,lvl in runs:
        if lvl==0 and dur>=SYNC_GAP_US:
            if cur: frames.append(cur); cur=[]
            continue
        cur.append((dur,lvl))
    if cur: frames.append(cur)
    return frames

def marks_from_runs(frame_runs):
    marks=[]
    for dur,lvl in frame_runs:
        if lvl==1: marks.append(dur)
    return marks

def estimate_short_long(marks):
    if not marks: return 300,1000
    avg=sum(marks)//len(marks)
    shorts=[m for m in marks if m<avg]
    longs=[m for m in marks if m>=avg]
    SHORT_US=int(sum(shorts)/len(shorts)) if shorts else min(marks)
    LONG_US=int(sum(longs)/len(longs)) if longs else max(marks)
    MID_US=(SHORT_US+LONG_US)//2
    return SHORT_US,LONG_US,MID_US

def marks_to_bits_auto(marks):
    SHORT_US,LONG_US,MID_US=estimate_short_long(marks)
    bits=[]
    for m in marks: bits.append('1' if m>=MID_US else '0')
    ones=bits.count('1'); zeros=bits.count('0')
    if ones>zeros:
        bits=['0' if b=='1' else '1' for b in bits]
        print("Auto-inverted bits (more 1s than 0s detected)")
    return ''.join(bits)

def bits_to_hexstr(bits):
    padded=bits
    while len(padded)%8!=0: padded+='0'
    out=[]
    for i in range(0,len(padded),8):
        byte=padded[i:i+8]
        out.append("{:02X}".format(int(byte,2)))
    return ' '.join(out)

# ----------------- Device recognition / learning -----------------
# Start with your Kerui door sensor ID
DEVICE_SIGNATURES = {
    "Kerui_Door": "1101000000000111",  # 0xD007
}

last_frame_bits_per_device = {}

def identify_or_learn(bits):
    # check known devices
    for name,sig in DEVICE_SIGNATURES.items():
        if sig in bits: return name
    # if unknown, learn new
    new_name="Unknown_{}".format(len(DEVICE_SIGNATURES))
    DEVICE_SIGNATURES[new_name]=bits[:20]  # store first 20 bits as signature
    print("Learned new device as '{}'".format(new_name))
    return new_name

# ----------------- Main processing -----------------
def process_and_print():
    global last_frame_bits_per_device
    runs=get_runs()
    print("Total edges captured:",edge_idx,"Collapsed runs:",len(runs))
    frames=split_frames(runs)
    print("Detected frames:",len(frames))
    for fi,f in enumerate(frames):
        marks=marks_from_runs(f)
        if not marks:
            print("Frame",fi,"has no mark runs")
            continue
        bits=marks_to_bits_auto(marks)
        device=identify_or_learn(bits)
        # frame repeat filtering per device
        last_bits=last_frame_bits_per_device.get(device,None)
        if bits==last_bits:
            print("Frame {} skipped (repeat) for device {}".format(fi,device))
            continue
        last_frame_bits_per_device[device]=bits
        hexs=bits_to_hexstr(bits)
        print("\n--- Frame {} --- Device: {} ---".format(fi,device))
        print("marks:",len(marks),"bits:",len(bits))
        print("bitstr:", bits[:256],"..." if len(bits)>256 else "")
        print("hex:",hexs)

# ----------------- Example usage -----------------
if __name__=="__main__":
    init_cc1101()
    time.sleep_ms(200)
    print("Ready. Trigger sensor to capture. Press Ctrl-C to stop capture and process.")
    try:
        start_capture()
        t0=time.ticks_ms()
        while capturing:
            time.sleep_ms(100)
            if time.ticks_diff(time.ticks_ms(),t0)>5000: stop_capture()
    except KeyboardInterrupt:
        stop_capture()
    process_and_print()
    try:
        with open("runs_dump.txt","w") as f:
            runs=get_runs()
            for dur,lvl in runs: f.write("{} {}\n".format(dur,lvl))
        print("Saved runs_dump.txt")
    except Exception as e:
        print("Failed writing runs_dump:",e)


