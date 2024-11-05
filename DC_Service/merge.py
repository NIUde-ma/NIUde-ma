import argparse
import os
import rosbag

def get_record_files(folder_path : str) -> list:
  record_files = []
  if folder_path == "":
    print("Folder path is empty!. Exit")
    exit(-1)

  if not os.path.exists(folder_path):
    print("Folder %s does not exist! Exit." % folder_path)
    exit(-1)

  for file in os.listdir(folder_path):
      if file.endswith('.bag'):
          record_files.append(file)
          print(file)

  print("Get %ld bag files." % len(record_files))
  return record_files


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
                    prog='ProgramName',
                    description='Merge multiple bag into one.')
  parser.add_argument('-d', '--data_folder')     
  parser.add_argument('-f', '--bag_name') 
  args = parser.parse_args()

  record_files = get_record_files(args.data_folder)
  if len(record_files) == 0:
    print("no bag file found in folde: %s" %(args.data_folder))
    exit(-1)

  sorted_record_files = sorted(record_files)
  out_file_name = sorted_record_files[0].split('.bag')[0] + "_merged.bag"
  if args.bag_name != None:
    out_file_name = args.bag_name[0]
  out_file_path = os.path.join(args.data_folder, out_file_name)
  print("out file path: %s" % out_file_path)

  with rosbag.Bag(out_file_path, mode='w') as outbag:
    for i in range(len(sorted_record_files)):
      print("writing %ld / %ld into bag: %s." %(i, len(sorted_record_files), out_file_name))
      in_file_name = sorted_record_files[i]
      in_file_path = os.path.join(args.data_folder, in_file_name)
      in_bag = rosbag.Bag(in_file_path)
      for topic, msg, t in in_bag.read_messages():
        outbag.write(topic, msg, t)
        pass