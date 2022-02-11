import os
import can
import paramiko
from datetime import datetime
import shutil


date = str()

class MySFTPClient(paramiko.SFTPClient):
    def put_dir(self, source, target):
        ''' Uploads the contents of the source directory to the target path. The
            target directory needs to exists. All subdirectories in source are
            created under target.
        '''
        for item in os.listdir(source):
            if os.path.isfile(os.path.join(source, item)):
                self.put(os.path.join(source, item), '%s/%s' % (target, item))
            else:
                self.mkdir('%s/%s' % (target, item), ignore_existing=True)
                self.put_dir(os.path.join(source, item), '%s/%s' % (target, item))

    def mkdir(self, path, mode=511, ignore_existing=False):
        ''' Augments mkdir by adding an option to not fail if the folder exists  '''
        try:
            super(MySFTPClient, self).mkdir(path, mode)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise

def log(message):
    with open('/home/pi/RobotWorkspaceSISK/log_update_data.txt', 'a') as file:
        file.write(message+'\n')

def update_date():
    global date
    date = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")

def send_file(source_path, target_path):

    transport = paramiko.Transport(('23.102.182.59', 22))
    transport.connect(username='siskproject', password='siskProject1324')
    sftp = MySFTPClient.from_transport(transport)
    sftp.mkdir(target_path, ignore_existing=True)
    sftp.put_dir(source_path, target_path)
    sftp.close()

if __name__ == '__main__':
    update_date()
    parent_dir = '/home/pi/CAN_Tester'
    directory = date
    path_of_logs = os.path.join(parent_dir, directory)
    mode = 0o755
    output_file = os.path.join(parent_dir, 'output.xml')
    debug_file = os.path.join(parent_dir, 'debug.log')
    report_file = os.path.join(parent_dir, 'report.html')
    log_file = os.path.join(parent_dir, 'log.html')
    target_path = os.path.join('/var/www/html', directory)
    files = [output_file, debug_file, report_file, log_file]
    os.mkdir(path_of_logs, mode)
    for i in files:
        shutil.copy(i, path_of_logs)

    send_file(path_of_logs, target_path)
    shutil.rmtree(path_of_logs)

