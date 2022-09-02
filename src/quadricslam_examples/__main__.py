from glob import glob
import os

if __name__ == '__main__':
    print("Please run with one of the following examples:")
    for e in [
            os.path.basename(p)
            for p in glob(os.path.join(os.path.dirname(__file__), '*.py'))
            if not p.endswith('__.py')
    ]:
        print("\t%s" % e[:-3])
    print("\n(e.g. 'python3 -m quadricslam_examples.hello_quadricslam')")
