<template>
  <div>
    <button class="button" @click="download_csv_file(spectral_data)">
      Generate Report
    </button>
  </div>
</template>

<script>
export default {
  props: {
    spectral_data: {
      type: Array,
      required: true,
    },
  },
  data() {
    return {
      csvFileData: [],
    };
  },
  methods: {
    download_csv_file: function (spectral_data) {
      const time = new Date(Date.now());
      const timeString =
        time.toTimeString().substring(0, 17) + " " + time.toDateString();
      for (let i = 0; i < spectral_data.length; i++) {
        this.csvFileData[i] = [timeString, spectral_data[i].toFixed(0)];
      }
      //define the heading for each row of the data
      var csv = "Timestamp,Spectral 0,Spectral 1,Spectral 2\n";
      //merge the data with CSV
      this.csvFileData.forEach(function (row) {
        csv += row.join(",");
        csv += "\n";
      });
      var hiddenElement = document.createElement("a");
      hiddenElement.href = "data:text/csv;charset=utf-8," + encodeURI(csv);
      hiddenElement.target = "_blank";
      //provide the name for the CSV file to be downloaded
      let report_name = prompt("Enter filename:", timeString);
      hiddenElement.download = report_name + ".csv";
      hiddenElement.click();
    },
  },
};
</script>

<style scoped>
button {
  width: 17vh;
  height: 7vh;
}
</style>
