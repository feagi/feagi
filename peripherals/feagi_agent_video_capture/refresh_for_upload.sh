for file in $(find . -name "*egg-info*");do
      sudo rm -r $file
done

for file in $(find . -name "dist");do
      sudo rm -r $file
done

