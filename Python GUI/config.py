class Config:
    BAUD_RATES = ['2400', '4800', '9600', '14400', '115200']
    DEFAULT_BAUD = '115200'
    LOG_EXPORT_DIR = 'exports/'
    SERIAL_DATA_FORMAT = '<3i2b3x1B18x'
    MAX_LOG_LINES = 1000
    WINDOW_TITLE = 'Mobile Robot GUI'
    WINDOW_SIZE = '1200x800'
    PLOT_REFRESH_INTERVAL = 0.05
    ENV_FOLLOW_BUFFER = 10.0
    ENV_DEFAULT_BUFFER = 10.0
    EXPORT_FILE_PREFIX = 'serial_log_'
    EXPORT_FILE_SUFFIX = {
        'txt': '.txt',
        'csv': '.csv',
        'xml': '.xml'
    }
    PPS_TO_RADS_PER_SECOND = 100
