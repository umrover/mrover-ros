<template>
        <div>
          <table
          class="tableFormat"
          style="undefined;table-layout: fixed; width: 400px"
          >
            <colgroup>
            <col style="width: 20px" />
            <col style="width: 30px" />
            <col style="width: 30px" />
            </colgroup>
            
              <th class="tableElement tableHeader"  >{{ header }}</th>
             
                <th class="tableElement"  >
                    {{leftmotor}}
                </th >
                <th class="tableElement "  >
                    {{ rightmotor}}
                </th>
           
            
            </table>
        </div>

</template>

<script>    
export default {
  props: {
    header: {
      type: String,
      required: true,
    },

    leftmotor: {
      type: String,
      required: true,
    },

    rightmotor: {
      type: String,
      required: true,
    },

   
    
  },
};
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}

.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
  text-align: right;
  vertical-align: top;
}

.tableFormat {
  border-collapse: collapse;
  border-spacing: 0;
}

.tableFormat td {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 13px;
  overflow: hidden;
  padding: 10px 5px;
  word-break: normal;
}

.tableFormat th {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 13px;
  font-weight: normal;
  overflow: hidden;
  padding: 10px 5px;
  word-break: normal;
}

.tableFormat .tableElement {
  border-color: inherit;
  text-align: center;
  vertical-align: top;
  width: 30px;
}

.tableElement.tableHeader {
  font-weight: bold;
  width: 20px;

}
</style>
