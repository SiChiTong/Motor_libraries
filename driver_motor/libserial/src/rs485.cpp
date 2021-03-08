#include "rs485.h"

/*
 * Configure
 * @param devfile   Path name of dev  
 * @param baud      Baudrate 
 * @param parity    Parity
 * @param data_bit
 * @param stop_bit 
 */
int rs485::_rs485_connect(rs485_t &ctx)
{
    struct termios tios;
    int flags;
    /* The O_NOCTTY flag tells UNIX that this program doesn't want
       to be the "controlling terminal" for that port. If you
       don't specify this then any input (such as keyboard abort
       signals and so forth) will affect your process
       Timeouts are ignored in canonical input mode or when the
       NDELAY option is set on the file via open or fcntl 
	*/
    flags = O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY;
    ctx.s = open(ctx._port,flags);       
    if (ctx.s < 0) 
    {
        std::stringstream e;
        e << ctx._port <<" is failed: " << strerror(errno);
        error_msg = e.str();
        _connected = false;
        return -1;
    }
    else
    {
        fcntl(ctx.s, F_SETFL, 0);
    }
    /* Save */
    if(tcgetattr(ctx.s, &ctx._old_tios) <0)
    {
        ROS_ERROR("Can't get terminal parameters");
    }

    /*
    * Enable the receiver and set local mode...
    */
    bzero(&tios, sizeof(tios));
    
    /*  C_ISPEED     Input baud (new interface)
     *  C_OSPEED     Output baud (new interface)
     */
    speed_t speed = setBaudRate(ctx._baud);
    /* Set the baud rate */
    if ((cfsetispeed(&tios, speed) < 0) ||
        (cfsetospeed(&tios, speed) < 0)) 
    {
        close(ctx.s);
        ctx.s = -1;
        _connected = false;
        return -1;
    }
    setDataBits(tios,ctx._data_bit);
    setStopBits(tios,ctx._stop_bit);
    setParity(tios, ctx._parity);
    
    /* Read the man page of termios if you need more information.
     * This field isn't used on POSIX systems
     * tios.c_line = 0;
     */

    /* C_LFLAG      Line options
       ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
       ICANON       Enable canonical input (else raw)
       XCASE        Map uppercase \lowercase (obsolete)
       ECHO Enable echoing of input characters
       ECHOE        Echo erase character as BS-SP-BS
       ECHOK        Echo NL after kill character
       ECHONL       Echo NL
       NOFLSH       Disable flushing of input buffers after
       interrupt or quit characters
       IEXTEN       Enable extended functions
       ECHOCTL      Echo control characters as ^char and delete as ~?
       ECHOPRT      Echo erased character as character erased
       ECHOKE       BS-SP-BS entire line on line kill
       FLUSHO       Output being flushed
       PENDIN       Retype pending input at next read or input char
       TOSTOP       Send SIGTTOU for background output
       Canonical input is line-oriented. Input characters are put
       into a buffer which can be edited interactively by the user
       until a CR (carriage return) or LF (line feed) character is
       received.
       Raw input is unprocessed. Input characters are passed
       through exactly as they are received, when they are
       received. Generally you'll deselect the ICANON, ECHO,
       ECHOE, and ISIG options when using raw input
    */

    /* Raw input */
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);;

    /* Software flow control is disabled */
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* C_OFLAG      Output options
       OPOST        Postprocess output (not set = raw output)
       ONLCR        Map NL to CR-NL
       ONCLR ant others needs OPOST to be enabled
    */

    /* Raw ouput */
    tios.c_oflag = 0;

    /* C_CC         Control characters
       VMIN         Minimum number of characters to read
       VTIME        Time to wait for data (tenths of seconds)
       UNIX serial interface drivers provide the ability to
       specify character and packet timeouts. Two elements of the
       c_cc array are used for timeouts: VMIN and VTIME. Timeouts
       are ignored in canonical input mode or when the NDELAY
       option is set on the file via open or fcntl.
       VMIN specifies the minimum number of characters to read. If
       it is set to 0, then the VTIME value specifies the time to
       wait for every character read. Note that this does not mean
       that a read call for N bytes will wait for N characters to
       come in. Rather, the timeout will apply to the first
       character and the read call will return the number of
       characters immediately available (up to the number you
       request).
       If VMIN is non-zero, VTIME specifies the time to wait for
       the first character read. If a character is read within the
       time given, any read will block (wait) until all VMIN
       characters are read. That is, once the first character is
       read, the serial interface driver expects to receive an
       entire packet of characters (VMIN bytes total). If no
       character is read within the time allowed, then the call to
       read returns 0. This method allows you to tell the serial
       driver you need exactly N bytes and any read call will
       return 0 or N bytes. However, the timeout only applies to
       the first character read, so if for some reason the driver
       misses one character inside the N byte packet then the read
       call could block forever waiting for additional input
       characters.
       VTIME specifies the amount of time to wait for incoming
       characters in tenths of seconds. If VTIME is set to 0 (the
       default), reads will block (wait) indefinitely unless the
       NDELAY option is set on the port with open or fcntl.
    */
    /* Unused because we use open with the NDELAY option */
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 20;
    /* clean port */
  	tcflush(ctx.s, TCIFLUSH);
    /* activate the settings port */
    if (tcsetattr(ctx.s, TCSANOW, &tios) < 0) {
        ROS_ERROR("Can't get terminal parameters");
        close(ctx.s);
        ctx.s = -1;
        _connected = false;
        return -1;
    }
    _connected = true;
    return 0;
}

/*
 *  Reconnect to device if rs485 modules was disconected.
 */
void rs485::reconnect(void)
{
    _rs485_connect(ctx);
}

/*
 * create new port
 * @param devfile   Path name of dev  
 * @param baud      Baudrate 
 * @param parity    Parity
 * @param data_bit
 * @param stop_bit 
 */
void rs485::new_port(const char* devfile,unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit)
{
    ctx._port = devfile;
    ctx._baud = baud;
    ctx._parity = parity;
    ctx._data_bit = data_bit;
    ctx._stop_bit = stop_bit;
    _connected = false;
    if(parity == PARITY_EVEN || parity == PARITY_ODD)
        size_pkg++;
    switch(stop_bit)
    {
        case STOPBIT_1:
            size_pkg++;
            break;
        case STOPBIT_2:
            size_pkg +=2;
            break;
        default:
            break;
    }

    switch(data_bit)
    {
        case DATABIT_5:
            size_pkg += 5;
            break;
        case DATABIT_6:
            size_pkg += 6;
            break;
        case DATABIT_7:
            size_pkg += 7;
            break;
        case DATABIT_8:
            size_pkg += 8;
            break;
        default:
            break;
    }
    size_pkg +=2;  // bit start and stop

    switch(baud)
    {
    case 19200:
        param_safety = 1.5;
        break;
    case 115200:
        param_safety = 3.5;
        break;
    default:
        param_safety = 1.5;
        break;
    }
}

/*
 * Close serial port
 */
void rs485::close_port()
{
}

/*
 * Data Sender
 * @param to_send Request to Be Sent to Server
 * @param length  Length of the Request
 * @return        Size of the request
 */
ssize_t rs485::sendMsgs(uint8_t *to_send, uint16_t length)
{
    float time = param_safety*1000000/(ctx._baud / (size_pkg * length));
    //for(int i = 0; i < length; i++) printf("0x%x ",to_send[i]);
    //printf("\n");
    usleep(time);
    memset((to_send + length),'\0',1);
    ssize_t num = write(ctx.s, to_send, (size_t)length);
    return num;
}

/*
 * Data Receiver
 * @param buffer Buffer to Store the Data` Retrieved
 * @return       Size of Incoming Data
 */
ssize_t rs485::receiveMsgs(uint8_t *buffer) const 
{
    memset(buffer,'\0',sizeof(buffer));
    return read(ctx.s, (char *) buffer,MAX_MSG_LENGTH);
}

/*
 * @brief Constructor 
 * @param[in] devfile   Example /dev/tty* 
 * @param[in] baud      Number of transmitted bits  per a second
 * @param[in] parity    The parity check bit {Even, None , Old }
 * @param[in] data_bit  Number of bits in a transmission frame 
 * @param[in] stop_bit  End bit
 */
rs485::rs485(const char* devfile, unsigned int baud, parity_t parity, data_bits_t data_bit,stop_bits_t stop_bit)
{
    new_port(devfile, baud, parity, data_bit, stop_bit);
    _rs485_connect(ctx);
}

/*
 * Destructor.
 */
rs485::~rs485()
{
   
}